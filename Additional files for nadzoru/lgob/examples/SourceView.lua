#! /usr/bin/env lua5.1

require('lgob.loader') -- completion module??
require('lgob.gtksourceview')

local window = gtk.Window.new()
window:set('title', 'GtkSourceView demo', 'window-position',
	gtk.WIN_POS_CENTER, 'width-request', 400, 'height-request', 400)
window:connect('delete-event', gtk.main_quit)

local view    = gtk.SourceView.new()
local buffer  = view:get('buffer')
local manager = gtk.source_language_manager_get_default()
local lang    = manager:get_language('lua')

view:set('show-line-numbers', true, 'highlight-current-line', true, 'auto-indent', true)

buffer:set('text', [[
require('lgob.gtk')

-- create
local win = gtk.Window.new()
win:connect('delete-event', gtk.main_quit)
win:show_all()

-- run
gtk.main()
]], 'language', lang)

local scroll = gtk.ScrolledWindow.new()
scroll:set('hscrollbar-policy', gtk.POLICY_AUTOMATIC, 'vscrollbar-policy', gtk.POLICY_AUTOMATIC)

scroll:add(view)
window:add(scroll)

window:show_all()
gtk.main()
