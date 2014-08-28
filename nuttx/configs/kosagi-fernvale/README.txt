Fernvale
======

  This README file discusses the port of NuttX to the Fernvale board.

Contents
========

  o Development Environment
  o IDEs
  o NuttX buildroot Toolchain
  o Boot Sequence

Development Environment
=======================

  This port was designed to be compiled on Novena, using the native toolchain.
  The native toolchain is hardfloat, so you need your own libgcc.a that 
  supports soft-float.


IDEs
====

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.

  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/arm/src/lpc31xx,
     arch/arm/src/common, arch/arm/src/arm, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lpc31xx/lpc31_vectors.S.  You may have to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by an IDE.

NuttX buildroot Toolchain
=========================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh olimex-lpc-h3131/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/arm926t-defconfig-4.2.4 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

Boot Sequence
=============

  Currently, Fernvale requires a serial bootstrap.  This program is called
  fernly-loader, and will blast an image to offset 0x0.

