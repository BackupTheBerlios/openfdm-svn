rm -rf autom4te.cache
rm -f src/OpenFDM/OpenFDMConfig.h.in src/OpenFDM/config.h.in
touch src/OpenFDM/OpenFDMConfig.h.in src/OpenFDM/config.h.in
libtoolize
aclocal
autoconf
automake --add-missing
