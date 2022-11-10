#! /usr/bin/env lua5.1

require('lgob.gtk')

local window = gtk.Window.new(gtk.WINDOW_TOPLEVEL)
window:connect('delete-event', gtk.main_quit)
window:set('title', 'Entry demo', 'window-position', gtk.WIN_POS_CENTER, 'default-width', 200)

local entry = gtk.Entry.new()
entry:set('primary-icon-stock'  , 'gtk-find')
entry:set('secondary-icon-stock', 'gtk-clear')

entry:connect('icon-press', function(ud, pos, event)
    if pos == gtk.ENTRY_ICON_PRIMARY then
        print('searching for ' .. entry:get('text') )
    else
        entry:set('text', '')
    end
end)

window:add(entry)
window:show_all()

gtk.main()
