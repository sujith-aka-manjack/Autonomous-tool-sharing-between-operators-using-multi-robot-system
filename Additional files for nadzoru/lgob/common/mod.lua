mod = {
    name = 'common',
    src  = 'interface.c'
}

init   (mod)
compile(mod)
install(mod)
clean  (mod)
install('${MOD}/src/types.h ${DEST}/include/lgob/common/types.h' )
