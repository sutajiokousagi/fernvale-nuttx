nuttx			osmocomBB
common			src/target/firmware
include			include/


Modified:
---------
nuttx			osmocomBB
layer1/main.c		apps/layer1/main.c
board/*/init.c		apps/layer1/board/init.c

Deleted:
--------
osmocomBB/include/keypad.h	-> Useless: we want to use nuttx's keypad infrastructure instead
osmocomBB/include/stdint.h	-> To use nuttx's one
osmocomBB/include/memory.h	-> To use compat.h instead

Already in nuttx:
-----------------
osmocomBB						nuttx				apps

msgb.h, linuxlist.h:
src/shared/libosmocore/include/osmocom/core/		misc/tools/osmocon/		apps/osmocomBB/libosmocore/include/osmocom/core
