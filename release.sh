git pull
VER=`date +"%y.%m%d-%H%M"`
comment=$1
touch ./main/Version.h
idf.py build
cp ./build/xcvario_pro.bin  ./images/xcvario_pro-${VER}.bin
git add ./images/xcvario_pro-${VER}.bin
git commit -a -m "new version: xcvario_pro-${VER}.bin; Changes: $comment"
git push
git tag -a v${VER} -m "$comment"
