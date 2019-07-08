SOURCES=$(wildcard *.c)
OBJECTS=$(SOURCES:%.c=%.o)
INCFILES=$(wildcard *.h)

LIBS = -lnanomsg -ldill -lrt -lpthread

ALL = sock0 sock1 readgps readgpsbus reader readerbus fly spider flies dill3 readgpsd

all: $(ALL)

sock: $(OBJECTS)
	$(CC) -o $@ $^ $(LIBS)

%.o: %.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f  *.o *~ ${ALL}

sock0: sock0.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

sock0.o: sock0.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

sock1: sock1.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

sock1.o: sock1.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

sock2: sock2.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

sock2.o: sock2.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

readgps: readgps.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

readgps.o: readgps.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

readgpsbus: readgpsbus.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

readgpsbus.o: readgpsbus.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

reader: reader.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

reader.o: reader.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

readerbus: readerbus.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

readerbus.o: readerbus.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

fly: fly.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

fly.o: fly.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

spider: spider.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

spider.o: spider.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

flies: flies.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

flies.o: flies.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

dill3: dill3.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

dill3.o: dill3.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@

readgpsd: readgpsd.o
	$(CC) -std=gnu99 -o $@ $^ $(LIBS)

readgpsd.o: readgpsd.c $(INCFILES)
	$(CC) $(CFLAGS) -c $< -o $@
