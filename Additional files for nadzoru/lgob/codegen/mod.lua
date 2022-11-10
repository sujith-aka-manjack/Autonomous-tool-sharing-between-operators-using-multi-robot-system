mod = {
    name = 'codegen'
}

init(mod)

install('${MOD}/bin/generator.lua      ${DEST}/bin/lgob-generator'     )
install('${MOD}/bin/gir-parser.lua     ${DEST}/bin/lgob-gir-parser'    )
install('${MOD}/bin/update-version.lua ${DEST}/bin/lgob-update-version')
chmod  ('${DEST}/bin/lgob-generator'                                   )
chmod  ('${DEST}/bin/lgob-gir-parser'                                  )
chmod  ('${DEST}/bin/lgob-update-version'                              )

mkdir  ( '${DEST}/${SHARED}' )
install('${MOD}/src/lgob/*.lua         ${DEST}/${SHARED}'              )
