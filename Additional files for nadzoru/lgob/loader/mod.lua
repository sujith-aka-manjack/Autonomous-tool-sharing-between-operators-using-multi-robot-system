mod = {
    name = 'loader',
    src  = 'loader.c'
}

init   (mod)
compile(mod)
install('${MOD}/src/loader.lua    ${DEST}/${SHARED}/loader.lua')
install('${MOD}/src/loader.${EXT} ${DEST}/${LIB}/_loader.${EXT}')
clean  (mod)
