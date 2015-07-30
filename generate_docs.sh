set -e

# make build dir
if [ ! -d build ]
then
  mkdir build
fi
# make docs
cd build
cmake ..
make doc
cd ..
# clean docs folder
if [ ! -d docs ]
then
  mkdir docs
fi
if [ -d docs/html ]
then 
  rm -rf docs/html
fi
mkdir docs/html
# copy docs
cp -R build/doc/html/* docs/html/
# remove references to local folders
perl -pi -e 's/\/home.*include/include/g' docs/html/*.html
