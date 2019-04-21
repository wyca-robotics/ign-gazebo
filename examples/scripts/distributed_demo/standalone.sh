#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

$DIR/../../standalone/joy_to_twist/build/joy_to_twist $DIR/../../standalone/joy_to_twist/joy_to_twist.sdf &
$DIR/../../standalone/joystick/build/joystick $DIR/../../standalone/joystick/joystick.sdf &
ign-gazebo -v 4  --levels -f $DIR/standalone.sdf
