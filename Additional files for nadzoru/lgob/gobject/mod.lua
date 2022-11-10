mod = {
    name = 'gobject',
    pkg  = 'gobject-2.0'
}

init    (mod)
generate(mod)
compile (mod)
install (mod)
clean   (mod)
install('${MOD}/src/types.h ${DEST}/include/lgob/gobject/types.h')
