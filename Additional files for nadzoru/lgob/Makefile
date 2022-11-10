DESTDIR = /
PREFIX  = /usr

all:
	lua5.1 build_all.lua out

install:
	mkdir -p $(DESTDIR)/$(PREFIX)
	cp -r out/* $(DESTDIR)/$(PREFIX)

clean:
	rm -rf out
