#! /usr/bin/env lua5.1

--[[
	Path example.
--]]

require("lgob.gobject")

local file = "/home/myuser/Documents/hi.lua"

print(glib.path_is_absolute(file))
print(glib.path_skip_root(file))
print(glib.path_get_basename(file))
print(glib.path_get_dirname(file))
print(glib.find_program_in_path("lua"))
