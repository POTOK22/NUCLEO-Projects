#!/usr/bin/env python3
"""
ov7670_receiver.py - odbior i podglad obrazu z OV7670 przez STM32.

Wymagania:
    pip install pyserial opencv-python numpy

Uzycie:
    python ov7670_receiver.py --port COM5
    python ov7670_receiver.py --port COM5 --baud 2000000 --scale 3

Klawisze w oknie podgladu:
    q / ESC - wyjscie
    s       - zapis biezacej klatki do PNG
    b       - przelaczenie kolejnosci bajtow RGB565 (gdy kolory sa dziwne)

Konsola strojenia (wpisuj w TYM terminalu, Enter wysyla do plytki):
    ?          lista komend
    w 3d 80    zapis rejestru (hex): COM13 bez auto-zmniejszania nasycenia
    r c9       odczyt rejestru
    s 111      nasycenie macierzy kolorow w % (max 111)
    b 18       jasnosc, c 60 kontrast (hex)
    d          zrzut rejestrow koloru/ekspozycji
Komendy sa wysylane w przerwie miedzy ramkami (plytka nie odbiera w trakcie
wysylania klatki).
"""

import argparse
import queue
import sys
import threading
import time

import numpy as np
import serial

try:
    import cv2
except ImportError:
    sys.exit("Brak opencv-python. Zainstaluj: pip install opencv-python")

MAGIC = b"\xA5\x5AOV"          # 4 bajty naglowka ramki
HEADER_LEN = 10                # magic(4) + width(2) + height(2) + stride(2)
MAX_PIXELS = 1024 * 1024       # zabezpieczenie przed smieciami w naglowku


class Stream:
    """
    Buforowany odbior z portu. Czytamy duzymi kawalkami (nie bajt po bajcie -
    przy 2 Mbaud petla Pythona po 1 B nie nadaza i sterownik gubi dane),
    a naglowek ramki szukamy w buforze.
    """

    def __init__(self, ser):
        self.ser = ser
        self.buf = bytearray()
        self.lost_bytes = 0      # bajty "z ramki" znalezione poza ramka

    def _fill(self):
        """Dociaga to, co czeka w porcie (min. 1 B, z timeoutem portu)."""
        chunk = self.ser.read(max(1, self.ser.in_waiting))
        if chunk:
            self.buf.extend(chunk)
        return bool(chunk)

    def read_exactly(self, n):
        """Zwraca dokladnie n bajtow albo None przy timeoucie."""
        while len(self.buf) < n:
            if not self._fill():
                return None
        data = bytes(self.buf[:n])
        del self.buf[:n]
        return data

    def find_frame(self):
        """
        Szuka naglowka ramki. Wszystko przed nim to tekst diagnostyczny ze
        STM32 (drukujemy) albo resztki ramki po zgubionych bajtach (liczymy,
        nie drukujemy). Zwraca True gdy znalazl MAGIC, None przy timeoucie.
        """
        while True:
            idx = self.buf.find(MAGIC)
            if idx >= 0:
                self._emit_text(self.buf[:idx])
                del self.buf[: idx + len(MAGIC)]
                return True
            # zostaw ogon, w ktorym moze byc poczatek MAGIC
            keep = len(MAGIC) - 1
            if len(self.buf) > keep:
                self._emit_text(self.buf[:-keep])
                del self.buf[:-keep]
            if not self._fill():
                self._emit_text(self.buf)
                self.buf.clear()
                return None

    def _emit_text(self, data):
        if not data:
            return
        printable = bytes(b for b in data if b in (9, 10, 13) or 32 <= b < 127)
        junk = len(data) - len(printable)
        if junk:
            self.lost_bytes += junk
        if printable.strip():
            sys.stdout.write(printable.decode("ascii"))
            sys.stdout.flush()


def stdin_reader(cmd_queue):
    """Watek: kazda linia z klawiatury laduje w kolejce komend."""
    for line in sys.stdin:
        line = line.strip()
        if line:
            cmd_queue.put(line)


def rgb565_to_bgr(raw, width, height, stride, big_endian=True):
    """
    Konwersja bufora RGB565 na obraz BGR (format oczekiwany przez OpenCV).
    stride = bajtow na linie tak, jak przyszly z DMA (moze byc != width*2,
    np. 317 gdy skaler kamery obcina piksele) - bierzemy z kazdej linii
    pierwsze width*2 bajtow.
    """
    if len(raw) != stride * height:
        return None

    rows = np.frombuffer(raw, dtype=np.uint8).reshape((height, stride))
    rows = np.ascontiguousarray(rows[:, : width * 2])
    dtype = ">u2" if big_endian else "<u2"
    px = rows.view(dtype).astype(np.uint16).reshape((height, width))

    r = ((px >> 11) & 0x1F).astype(np.uint8)
    g = ((px >> 5) & 0x3F).astype(np.uint8)
    b = (px & 0x1F).astype(np.uint8)

    # rozciagniecie 5/6 bitow na pelne 8 bitow
    r = (r << 3) | (r >> 2)
    g = (g << 2) | (g >> 4)
    b = (b << 3) | (b >> 2)

    return np.dstack((b, g, r))


def main():
    ap = argparse.ArgumentParser(description="Podglad obrazu z OV7670")
    ap.add_argument("--port", required=True, help="np. COM5 lub /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--scale", type=int, default=3, help="powiekszenie podgladu")
    ap.add_argument("--smooth", action="store_true",
                    help="interpolacja przy powiekszaniu (mniej 'klockow')")
    ap.add_argument("--little-endian", action="store_true",
                    help="odwrotna kolejnosc bajtow RGB565")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=2)
    except serial.SerialException as exc:
        sys.exit(f"Nie moge otworzyc portu {args.port}: {exc}")
    # Domyslny bufor sterownika COM w Windows (~4 KB) to przy 2 Mbaud ledwie
    # 20 ms - tyle trwa cv2.imshow(). Bez tego gubimy bajty co klatke.
    try:
        ser.set_buffer_size(rx_size=4 * 1024 * 1024)
    except (AttributeError, ValueError):
        pass
    stream = Stream(ser)

    print(f"Nasluchuje na {args.port} @ {args.baud} baud. Ctrl+C konczy.")
    print("Wcisnij RESET (czarny przycisk) na Nucleo, zeby zobaczyc log startowy.")

    big_endian = not args.little_endian
    frames = 0
    t_start = time.time()
    last_img = None

    cmd_queue = queue.Queue()
    threading.Thread(target=stdin_reader, args=(cmd_queue,), daemon=True).start()

    try:
        while True:
            lost_before = stream.lost_bytes
            if stream.find_frame() is None:
                print("[i] Brak danych - czekam...")
                continue
            if frames and stream.lost_bytes != lost_before:
                print(f"[!] {stream.lost_bytes - lost_before} B smieci miedzy ramkami"
                      f" - PC gubi bajty (lacznie {stream.lost_bytes})")

            head = stream.read_exactly(HEADER_LEN - len(MAGIC))
            if head is None:
                continue

            width = head[0] | (head[1] << 8)
            height = head[2] | (head[3] << 8)
            stride = head[4] | (head[5] << 8)

            if (width == 0 or height == 0 or stride < width * 2
                    or width * height > MAX_PIXELS):
                print(f"[!] Podejrzany naglowek: {width}x{height}, stride={stride} - pomijam")
                continue

            raw = stream.read_exactly(stride * height)
            if raw is None:
                print("[!] Ramka urwana (timeout)")
                continue

            img = rgb565_to_bgr(raw, width, height, stride, big_endian)
            if img is None:
                continue
            if frames == 0:
                print(f"[+] Pierwsza ramka: {width}x{height}, stride={stride} B/linie")

            # plytka wlasnie skonczyla wysylac i czeka na kolejna klatke -
            # to jedyny moment, w ktorym na pewno odbiera z UART
            while not cmd_queue.empty():
                cmd = cmd_queue.get()
                ser.write((cmd + "\n").encode("ascii", "replace"))

            last_img = img
            frames += 1
            elapsed = time.time() - t_start
            fps = frames / elapsed if elapsed > 0 else 0.0

            interp = cv2.INTER_CUBIC if args.smooth else cv2.INTER_NEAREST
            view = cv2.resize(img, (width * args.scale, height * args.scale),
                              interpolation=interp)
            cv2.putText(view, f"{width}x{height} s={stride}  {fps:.1f} fps",
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1, cv2.LINE_AA)
            cv2.imshow("OV7670", view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and last_img is not None:
                name = time.strftime("ov7670_%Y%m%d_%H%M%S.png")
                cv2.imwrite(name, last_img)
                print(f"[+] Zapisano {name}")
            if key == ord("b"):
                big_endian = not big_endian
                print(f"[i] Kolejnosc bajtow: {'BE' if big_endian else 'LE'}")

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        cv2.destroyAllWindows()
        print(f"Odebrano {frames} klatek.")


if __name__ == "__main__":
    main()
