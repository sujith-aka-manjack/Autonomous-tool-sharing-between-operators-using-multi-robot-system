#! /usr/bin/env lua5.1

--[[
    Utility to same some repetitive work when releasing a new stable version.
    Usage: ./make_release [version, by default is the date] 
--]]

local function ex(...)
    local cmd = string.format(...)
    print('->', cmd)
    os.execute(cmd)
end
 
local name = arg[1] or os.date('lgob-%y.%m')
assert(not name:match('^%.') and not name:match('^/')) -- avoid disasters
ex('rm -rf out')
ex('rsync -a . %s --exclude %s --exclude .directory --exclude .git --exclude .gitignore --exclude res', name, name)
ex('tar -cvjf %s.tar.bz2 %s', name, name)
ex('md5sum %s.tar.bz2', name)
ex('rm -r %s', name)
