CC = gcc
SOURCE = envioSerial.c
TARGET = envioSerial

FILEH = audio.h
FILEMP3 = audio.mp3
FILERAW = audio.raw
SAMPLERATE = 22050

$(TARGET): $(SOURCE) $(FILEH) 
	$(CC) $(SOURCE) -o $(TARGET)

$(FILEH): $(FILERAW)
	xxd -i $(FILERAW) > $(FILEH)

$(FILERAW): $(FILEMP3)
	# comando para gerar arquivo raw
	ffmpeg -i $(FILEMP3) -ar $(SAMPLERATE) -ac 1 -f u8 $(FILERAW)

clean:
	rm $(TARGET) $(FILEH) $(FILERAW)

