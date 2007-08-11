#! /bin/sh

srcdir=`dirname $0`
test -z "$srcdir" && srcdir=.

ORIGDIR=`pwd`
cd $srcdir

rm -f $srcdir/src/OpenFDM/OpenFDMConfig.h.in $srcdir/src/OpenFDM/config.h.in
touch $srcdir/src/OpenFDM/OpenFDMConfig.h.in $srcdir/src/OpenFDM/config.h.in
# libtoolize
# aclocal
# autoconf
# automake --add-missing
autoreconf -v --install --symlink || exit $?

