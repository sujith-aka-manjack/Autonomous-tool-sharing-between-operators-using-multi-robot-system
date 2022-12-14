#! /usr/bin/env lua5.1

require('lgob.gtk')

local ui = [[
<?xml version="1.0" encoding="UTF-8"?>
<interface>
  <!-- interface-requires gtk+ 3.6 -->
  <object class="GtkWindow" id="window1">
    <property name="can_focus">False</property>
    <property name="title" translatable="yes">GtkStack</property>
    <child>
      <object class="GtkGrid" id="grid1">
        <property name="visible">True</property>
        <property name="can_focus">False</property>
        <child>
          <object class="GtkStackSwitcher" id="switcher">
            <property name="visible">True</property>
            <property name="stack">stack</property>
            <property name="halign">center</property>
          </object>
          <packing>
            <property name="left_attach">0</property>
            <property name="top_attach">0</property>
            <property name="width">1</property>
            <property name="height">1</property>
          </packing>
        </child>
        <child>
          <object class="GtkStack" id="stack">
            <property name="visible">True</property>
            <property name="can_focus">True</property>
            <property name="transition-type">crossfade</property>
            <child>
              <object class="GtkLabel" id="label1">
                <property name="visible">True</property>
                <property name="label">Ni hao ma?
Como vai você?</property>
                <property name="margin-top">20</property>
                <property name="margin-bottom">20</property>
              </object>
              <packing>
                <property name="name">page1</property>
                <property name="title" translatable="yes">Page 1</property>
              </packing>
            </child>
            <child>
              <object class="GtkCheckButton" id="checkbutton1">
                <property name="label" translatable="yes">Page 2</property>
                <property name="visible">True</property>
                <property name="can_focus">True</property>
                <property name="receives_default">False</property>
                <property name="xalign">0</property>
                <property name="draw_indicator">True</property>
                <property name="halign">center</property>
                <property name="valign">center</property>
              </object>
              <packing>
                <property name="name">page2</property>
                <property name="title" translatable="yes">Page 2</property>
              </packing>
            </child>
            <child>
              <object class="GtkSpinner" id="spinner1">
                <property name="visible">True</property>
                <property name="can_focus">False</property>
                <property name="halign">center</property>
                <property name="valign">center</property>
                <property name="active">True</property>
              </object>
              <packing>
                <property name="name">page3</property>
                <property name="icon-name">face-laugh-symbolic</property>
              </packing>
            </child>
          </object>
          <packing>
            <property name="left_attach">0</property>
            <property name="top_attach">1</property>
            <property name="width">1</property>
            <property name="height">1</property>
          </packing>
        </child>
      </object>
    </child>
  </object>
</interface>
]]

local builder = gtk.Builder.new_from_string(ui, #ui)
local  window = builder:get_object("window1")
window:show_all()
window:connect('delete-event', gtk.main_quit)

gtk.main()
