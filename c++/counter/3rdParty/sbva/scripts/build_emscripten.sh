rm -rf CMake*
rm -rf src
rm -rf cmake*
rm -rf Make*
rm -rf sbva*
emcmake cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX=$EMINSTALL ..
emmake make -j$(nproc)
emmake make install
