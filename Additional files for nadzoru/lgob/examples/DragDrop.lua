#! /usr/bin/env lua5.1

require('lgob.gdk')
require('lgob.gtk')

local win   = gtk.Window.new()
local box   = gtk.Box.new(gtk.ORIENTATION_HORIZONTAL)
local entry = gtk.Entry.new()
entry:set_text('Hello drag')
local label = gtk.Label.new("Drag here")
box:add(entry, label)
win:add(box)

-- setup drag and drop
entry:drag_source_set(gdk.BUTTON1_MASK,
    { 'STRING', { target = 'text/plain', info = 12 }, },
    gdk.ACTION_COPY
)

label:drag_dest_set(gtk.DEST_DEFAULT_ALL,
    { 'STRING',
        { target = 'text/plain'   , info = 12 },
        { target = 'text/html'    , info = 13 },
        { target = 'text/uri-list', info = 14 },
    },
    gdk.ACTION_COPY
)

label:connect("drag-data-received", function(udata, context, x, y, data, info, time)
    local txt = gtk.SelectionData.get_text(data)
    print( string.format('x: %d, y: %d. Text: %s', x, y, txt) )
end)

win:set('title', 'Drag & Drop example')
win:connect('delete-event', gtk.main_quit)
win:show_all()
gtk.main()

