#! /usr/bin/env lua5.1

local lfs = require('lfs')

-- modules to parse a gir file and a devhelp file
local modules = {
    ['atk'] = {
        gir     = 'Atk-1.0.gir',
        devhelp = 'atk.devhelp2'
    },
    
    ['pixbuf'] = {
        gir       = 'GdkPixbuf-2.0.gir',
        devhelp   = 'gdk-pixbuf.devhelp2',
        noinstall = true,
    },
    
    ['gdk'] = {
        gir     = 'Gdk-3.0.gir',
        devhelp = 'gdk3.devhelp2'
    },
    
    ['gtk'] = {
        gir     = 'Gtk-3.0.gir',
        devhelp = 'gtk3.devhelp2'
    },
    
    ['gtksourceview'] = {
        gir     = 'GtkSource-3.0.gir',
        devhelp = 'gtksourceview-3.0.devhelp2'
    },
    
    ['pango'] = {
        gir     = 'Pango-1.0.gir',
        devhelp = 'pango.devhelp2'
    },
    
    ['pangocairo'] = {
        gir     = 'PangoCairo-1.0.gir',
        devhelp = 'pango.devhelp2'
    },
    
    ['vte'] = {
        gir     = 'Vte-2.90.gir',
        devhelp = 'vte-2.90.devhelp2'
    },
    
    ['poppler'] = {
        gir     = 'Poppler-0.18.gir',
        devhelp = 'poppler.devhelp2',
    },
    
    ['webkit'] = {
        gir     = 'WebKit-3.0.gir',
    },
}

local ex = function(cmd) print('----') print(cmd) print('----') os.execute(cmd) end
local sf = string.format
ex('mkdir -p def1 def2')

if lfs.attributes('gir-devhelp', 'mode') ~= 'directory' then
    ex('mkdir gir-devhelp')
    lfs.chdir('gir-devhelp')
    ex('tar xf ../gir-devhelp.tar.bz2')
    lfs.chdir('..')
end

-- gir -> def
for name, mod in pairs(modules) do
    ex( sf('lgob-gir-parser -i `pwd`/gir-devhelp/%s -o `pwd`/def1/%s.def -n %s', 
        mod.gir, name, name ) )
end

-- def -> versioned def
lfs.chdir('../codegen/src/')
local p = '../../res'

for name, mod in pairs(modules) do
    if mod.devhelp then
        ex( sf('../bin/update-version.lua %s/gir-devhelp/%s < %s/def1/%s.def > %s/def2/%s.def', p, 
            mod.devhelp, p, name, p, name ) )
    else
        ex( sf('cp %s/def1/%s.def %s/def2/%s.def', p, name, p, name) )
    end
end

-- install in the folders
lfs.chdir(p)

for name, mod in pairs(modules) do
    if not mod.noinstall then
        ex( sf('cp def2/%s.def ../%s/src/', name, mod.dir or name) )
    end
end

-- fix for GdkPixbuf
ex('tail -n +3 def2/pixbuf.def > ../gdk/src/pixbuf.def')

-- fix for GtkSourceView
ex('echo "defLib = \'Gtk\'" > ../gtksourceview/src/gtksourceview.def')
ex('tail -n +2 def2/gtksourceview.def >> ../gtksourceview/src/gtksourceview.def')

-- fix for WebkitGTK
ex([[sed --in-place "/since = '3.0'/d" ../webkit/src/webkit.def]])

-- clean
ex('rm -r def1 def2')

print('Do not forget to add "<include name="lgob" version="1.0"/>" in "GLib-2.0.gir" file')
