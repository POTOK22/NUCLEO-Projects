%% OV7670 -> STM32 -> UART -> MATLAB: podglad obrazu i krawedzie Canny'ego
%  Wymagania: MATLAB R2019b+ (serialport), Image Processing Toolbox (edge).
%
%  Ramka z plytki (patrz SendFrame w Core/Src/main.c):
%     [A5 5A 'O' 'V'] [W lo,hi] [H lo,hi] [S lo,hi] [H*S bajtow RGB565]
%  W = piksele w linii, H = linie, S = stride (bajty na linie, moze byc
%  wiekszy niz 2*W). Piksel RGB565: starszy bajt pierwszy.
%  Tekst pomiedzy ramkami to logi z plytki - trafiaja do konsoli.

clear; clc; close all;

%% Konfiguracja
PORT  = "COM9";
BAUD  = 2000000;
MAGIC = uint8([hex2dec('A5') hex2dec('5A') 'O' 'V']);

TH    = [0.05 0.15];   % progi Canny'ego [dolny gorny]
SIGMA = 2;             % rozmycie Gaussa przed detekcja

%% Inicjalizacja
s = serialport(PORT, BAUD, "Timeout", 5);
flush(s);
cleanup = onCleanup(@() clear('s'));   % zwolnij port takze po Ctrl+C

fig = figure('Name', 'OV7670 (zamknij okno, aby zakonczyc)', 'NumberTitle', 'off');
fprintf('Czekam na ramki na %s @ %d baud...\n', PORT, BAUD);

buf = uint8([]);
frames = 0;
tStart = tic;

%% Petla glowna
while ishandle(fig)
    [frame, buf] = nextFrame(s, buf, MAGIC);
    if isempty(frame)
        continue;
    end

    gray  = rgb565ToGray(frame);
    edges = edge(gray, 'canny', TH, SIGMA);

    frames = frames + 1;
    imshow([gray, uint8(edges) * 255]);
    title(sprintf('Obraz z kamery  |  Filtr: Operator Canny      %dx%d, %.1f fps', ...
          frame.width, frame.height, frames / toc(tStart)));
    drawnow limitrate;
end

fprintf('Zakonczono, odebrano %d ramek.\n', frames);

%% Funkcje lokalne

function [frame, buf] = nextFrame(s, buf, magic)
% Zwraca kolejna ramke (pola: width, height, stride, data) albo [] po
% timeoucie. Dane z portu sa czytane duzymi kawalkami do bufora buf.
    HDR = numel(magic) + 6;
    frame = [];
    while true
        idx = strfind(buf, magic);
        if isempty(idx)
            % brak naglowka - wypisz tekst, zostaw ogon na wypadek
            % naglowka przecietego granica kawalka
            keep = min(numel(buf), numel(magic) - 1);
            printText(buf(1:end-keep));
            buf = buf(end-keep+1:end);
        else
            printText(buf(1:idx(1)-1));
            buf = buf(idx(1):end);
            if numel(buf) >= HDR
                w  = double(buf(5)) + 256 * double(buf(6));
                h  = double(buf(7)) + 256 * double(buf(8));
                st = double(buf(9)) + 256 * double(buf(10));
                n  = h * st;
                if w == 0 || h == 0 || st < 2 * w || n > 4e6
                    buf = buf(2:end);   % to nie byl naglowek
                    continue;
                end
                if numel(buf) >= HDR + n
                    frame = struct('width', w, 'height', h, 'stride', st, ...
                                   'data', buf(HDR+1 : HDR+n));
                    buf = buf(HDR+n+1 : end);
                    return;
                end
            end
        end
        chunk = read(s, max(1, s.NumBytesAvailable), "uint8");
        if isempty(chunk)
            return;                     % timeout
        end
        buf = [buf, uint8(chunk)];
    end
end

function printText(bytes)
% Wypisuje logi z plytki linia po linii; bajty spoza ASCII pomija.
    persistent line
    if isempty(line), line = ''; end
    for b = bytes(:)'
        if b == 10
            if any(line > ' ')
                fprintf('%s\n', line);
            end
            line = '';
        elseif b >= 32 && b < 127
            line(end+1) = char(b);
        end
    end
end

function gray = rgb565ToGray(frame)
% RGB565 -> obraz uint8 w skali szarosci.
    rows = reshape(frame.data, frame.stride, frame.height)';   % H x S
    rows = rows(:, 1 : 2 * frame.width);
    px = bitshift(uint16(rows(:, 1:2:end)), 8) + uint16(rows(:, 2:2:end));

    R = uint8(bitshift(bitand(px, uint16(hex2dec('F800'))), -11) * 8);
    G = uint8(bitshift(bitand(px, uint16(hex2dec('07E0'))),  -5) * 4);
    B = uint8(bitand(px, uint16(hex2dec('001F'))) * 8);

    gray = rgb2gray(cat(3, R, G, B));
end
