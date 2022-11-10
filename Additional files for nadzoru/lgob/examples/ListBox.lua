#! /usr/bin/env lua5.1

require('lgob.gtk')

local win = gtk.Window.new()
win:set('width-request', 300, 'window-position', gtk.WIN_POS_CENTER)
    
local list = gtk.ListBox.new()
list:add(gtk.Label.new('Zebra')                 )
list:add(gtk.Label.new('Bus')                 )
list:add(gtk.Label.new('Parrot')                 )
list:add(gtk.Button.new_with_label("Hercules")     )
list:add(gtk.CheckButton.new_with_label("Sparta"))

-- some rows will be filtered
for i = 6, 20 do list:add(gtk.Label.new('Yippie-Kai-Yay ' .. i)) end

list:connect('row-selected', function(ud, row)
    if row then
        local id = row:get_index()
        print(string.format('Row %d selected!', id))
    end
end)

list:set_header_func(function(ud, row, before)
    local idx = row:get_index()
    if idx % 2 == 0 and not row:get_header() then
        local l = gtk.Label.new("<b>Header</b>")
        l:set_use_markup(true)
        row:set_header(l)
    end
end)

list:set_filter_func(function(ud, row)
    return row:get_index() < 6 
end)

list:set_sort_func(function(ud, row1, row2)
    local t1 = row1:get_child():get_label()
    local t2 = row2:get_child():get_label()

    if t1 < t2 then
        return -1
    elseif t1 > t2 then
        return 1
    else
        return 0
    end
end)

win:add(list)
win:connect('delete-event', gtk.main_quit)
win:show_all()
gtk.main()
