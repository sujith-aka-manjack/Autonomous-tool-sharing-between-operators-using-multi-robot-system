mod = {
    name = 'gst',
    src  = 'interface.c',
    pkg  = 'gstreamer-interfaces-0.10'
}

init   (mod)
compile(mod)
install(mod)
clean  (mod)
