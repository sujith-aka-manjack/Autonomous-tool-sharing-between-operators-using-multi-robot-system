#! /usr/bin/env lua5.1

--[[
    Usage: ./build_all.lua output_path
--]]

require('config')

local modules = {
    'codegen',
    'common',
    'loader',
    'gobject',
    'cairo',
    'atk',
    'pango',
    'pangocairo',
    'gdk',
    'gtk',
    'gtksourceview',
    'gst',
    'poppler',
    'vte',
    'webkit',
}

local out = assert(arg[1], './build_all.lua output_path')

for i = 1, #modules do
    os.execute(string.format('%s build.lua %s %s', LUA_EX, modules[i], out))
end
