OpenFX-OpenCV [![Build Status](https://api.travis-ci.org/devernay/openfx-opencv.png?branch=master)](https://travis-ci.org/devernay/openfx) [![Coverage Status](https://coveralls.io/repos/devernay/openfx-opencv/badge.png?branch=master)](https://coveralls.io/r/devernay/openfx?branch=master) [![Bitdeli Badge](https://d2weczhvl823v0.cloudfront.net/devernay/openfx-opencv/trend.png)](https://bitdeli.com/free "Bitdeli Badge")
=============

A set of OFX plugins (for Nuke, Sony Vegas, Scratch, etc.) which perform image processing using OpenCV

Compiling requires a recent installation of OpenCV

The opencv2fx plugins (inpaint and segment) were written by Bernd Porr <http://www.berndporr.me.uk/opencv2fx/>,
see opencv2fx/README for more information.

### Compiling (Unix/Linux/FreeBSD/OS X, using Makefiles)

On Unix-like systems, the plugins can be compiled by typing in a
terminal:
- `make [options]` to compile as a single combined plugin (see below
  for valid options).
- `make nomulti [options]` to compile as separate plugins (useful if
only a few plugins are is needed, for example). `make` can also be
executed in any plugin's directory.

The most common options are `CONFIG=release` to compile a release
version, `CONFIG=debug` to compile a debug version. Or
`CONFIG=relwithdebinfo` to compile an optimized version with debugging
symbols.

Another common option is `BITS=32`for compiling a 32-bits version,
`BITS=64` for a 64-bits version, and `BITS=Universal` for a universal
binary (OS X only).

See the file `Makefile.master`in the toplevel directory for other useful
flags/variables.

The compiled plugins are placed in subdirectories named after the
configuration, for example Linux-64-realease for a 64-bits Linux
compilation. In each of these directories, a `*.bundle` directory is
created, which has to be moved to the proper place
(`/usr/OFX/Plugins`on Linux, or `/Library/OFX/Plugins`on OS X), using
a command like the following, with the *same* options used for
compiling:

	sudo make install [options]
