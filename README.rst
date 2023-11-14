.. contents:: **Table of Contents**

Arduino Zero Stage Metronome
----------------------------

I created this metronome for use with my in-ear monitors during live performances.
Using a "regular" metronome (e.g. Korg TM60) is problematic, since you have to
stop and dial in the next BPM for each song, which slows down the set, and that
strategy also won't work for songs with tempo changes. I needed something that
would allow me to save multiple named presets for different songs, and switch
between them seamlessly by pressing a single button. This metronome project fills
that gap.

I am aware that many android/iOS metronome apps exist, and one may even exist that
provides all the features I need. However, I don't like the idea of using my phone
as a live/stage metronome, because if I forget to disable notifications / enable
airplane mode / do something to prevent phone calls and other sounds happening,
then I could get a call in the middle of a song and lose the metronome. Phones
are general-purpose devices, and there are many reasons that a phone might decide to
pause/mute the audio from your metronome app for something that it thinks is more
important. I want a dedicated metronome.

Features
========

* Uses high-quality 44.1KHz click sounds recorded directly from a Korg TM60 digital
  metronome (Stored on Arduino Zero internal flash, no external storage required).

* Supports up to 512 BPM (beats per minute), and up to 16 beats per bar.

* Supports creating & saving up to 128 presets. Each preset consists of a BPM
  value, a beats-per-bar value, and a string name that you choose yourself
  (Stored on Arduino Zero internal flash, no external storage required).

* Supports browsing/selecting presets by name.

* Supports switching to the next preset with a single button push, seamlessly
  (no matter when you press the button, the preset change will not be applied until
  the first beat of the next bar).

* Can be fully remotely/programatically controlled; all button presses can be "emulated"
  by sending serial commands to the Arduino Zero's serial port.

* Supports reading & writing saved preset data via the Arduino Zero's serial port.
  This makes it easy to migrate all your saved presets to another device.

* Arduino Zero internal flash (where all preset data is stored) is *only* written
  to when the device is powered off with a toggle switch, and only if any preset
  data has changed, in order to maximize the life of the device since the Arduino
  Zero internal flash has a limited number of write cycles.

Hardware Required
=================

* `Arduino Zero <https://store.arduino.cc/products/arduino-zero>`_ (Must be an Arduino Zero. Uno or other boards will not work.)
* `Adafruit 20x4 character LCD screen <https://www.adafruit.com/product/198>`_
* `HiLetgo PCM5102 I2S DAC <https://www.amazon.com/HiLetgo-Lossless-Digital-Converter-Raspberry/dp/B07Q9K5MT8>`_
* 8 pushbuttons, of whichever kind you want
* 1 toggle switch, of whichever kind you want

Wiring Diagram
==============

This diagram shows how all the components should be wired up to the Arduino Zero:

.. image:: images/wiring_diagram.drawio.png

Backing up / transferring saved presets
=======================================

This section describes how to transfer your saved presets to another Arduino Zero metronome.
You will need to have Python 3x installed, and you will also need to install the ``pyserial``
python library (e.g. ``pip install pyserial``). You also need to connect the Arduino Zero's
programming port to your computer via USB cable (same USB port that you use to program
sketches onto the Arduino Zero).

Downloading saved presets from an Arduino Zero metronome
########################################################

Use the ``scripts/preset_backup.py`` script with the ``save`` command to download saved
presets from a connected Arduino Zero metronome. The following command downloads saved
presets from an Arduino Zero metronome connected to COM14, and saves the downloaded
preset data in a file called ``saved_presets.txt``:

.. code::

    $ python scripts/preset_backup.py save COM14 saved_presets.txt

    Found 'Arduino Zero Stage Metronome 0.0.1' on COM14
    Downloading 12 presets
    12 preset(s) saved in 'saved_presets.txt'

Loading downloaded presets onto an Arduino Zero metronome
#########################################################

Use the ``scripts/preset_backup.py`` script with the ``load`` command to send downloaded
presets to a connected Arduino Zero metronome. The following command reads downloaded
presets from a file called ``saved_presets.txt`` and sends them to to an Arduino Zero
metronome connected to COM14:

.. code::

    $ python scripts/preset_backup.py load COM14 saved_presets.txt

    Found 'Arduino Zero Stage Metronome 0.0.1' on COM14
    Succesfully loaded 12 new presets to metronome

    Power off the metronome via toggle switch or via CLI 'off' command

NOTE: After loading presets, whenever you want to power off the metronome, it is important
to power off the metronome via the toggle switch, OR via the CLI 'off' command. If power is
removed unexpectedly, then the presets you just loaded will not be saved.

Usage
=====

The following section describes what each button does on each screen of the metronome.

Metronome screen
################

This is the screen that will be showing on power-on.

.. image:: images/metronome_screen.drawio.png

* **Up button**: Increases BPM
* **Down button**: Decreases BPM
* **Left button**: Decreases number of beats per bar
* **Right button**: Increases number of beats per bar
* **Middle button**: Start/stop metronome
* **Mode button**: Switch to preset playback screen
* **Add/Delete button**: Save preset (switches to name entry screen)

Preset playback screen
######################

This screen is used to edit/delete/play previously saved presets. This screen can
be reached by pressing the **Mode** button when on the metronome screen.

.. image:: images/preset_playback_screen.drawio.png

* **Up button**: Switch to next preset
* **Down button**: Switch to previous preset
* **Left button**: Nothing
* **Right button**: Nothing
* **Middle button**: Start/stop metronome
* **Mode button**: Switch to metronome screen
* **Add/Delete button**: Edit or delete preset (shows two options to select, "Edit" or "Delete")

Preset edit screen
##################

This screen is used to edit a previously saved preset. This screen can be reached by
pressing the **Add/Delete** button when on the preset playback screen, and then selecting "Edit".

.. image:: images/preset_edit_screen.drawio.png

* **Up button**: Increases BPM
* **Down button**: Decreases BPM
* **Left button**: Decreases number of beats per bar
* **Right button**: Increases number of beats per bar
* **Middle button**: Start/stop metronome
* **Mode button**: Switch to metronome screen (changes will not be saved)
* **Add/Delete button**: Save changes to preset (switches to preset playback screen)

Name entry screen
#################

This screen is used to enter a string to be used a name for a saved preset. This screen
can be reached by pressing the **Add/Delete** button when on the metronome screen.

.. image:: images/name_entry_screen.drawio.png

* **Up button**: Move cursor up
* **Down button**: Move cursor down
* **Left button**: Move cursor left
* **Right button**: Move cursor right
* **Middle button**: Select letter under cursor
* **Mode button**: Switch to metronome screen without saving preset
* **Add/Delete button**: Save changes to preset and switch back to metronome screen
                         (you can also select the asterisk **\*** with the cursor
                         to save and return to the metronome screen)
