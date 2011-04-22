#
# Makefile
#
# Top-level build for RFStompbox, a firmware for a USB guitar pdeal based
# on vusb.
#
# Copyright (C) 2011 Daniel Thompson <daniel@redfelineninja.org.uk> 
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#

all flash fuse readcal clean :
	$(MAKE) -C firmware $@


VERSION = $(shell grep '[0-9]:$$' ChangeLog | head -n 1 | cut -f1 -d:)
NAME = rfstompbox-$(VERSION)
FILES = $(shell git ls-files | grep -v .gitignore)
dist :
	tar --transform=s:^:$(NAME)/: -zcf ../$(NAME).tar.gz $(FILES) 
