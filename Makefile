###
#  Copyright (C) 2023 - Innovusion Inc.
#
#  All Rights Reserved.
#
#  $Id$
##
.PHONY: build all src example get_pcd clean

build: src

all: build

src:
	$(MAKE) -C src

example: | src
	$(MAKE) -C apps/example

get_pcd: | src
	$(MAKE) -C apps/tools/get_pcd


clean:
	$(MAKE) -C src clean
	$(MAKE) -C apps/example clean
	$(MAKE) -C apps/tools/get_pcd clean
