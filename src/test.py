
move()

main loop{

do move according to status


}

callback loop{

if init
    get center
    set status according to center

if y > 3.0m
    status 1:
            move drone till y=3
            get center
            set status according to center

if y < 3.5 and y > 2.0m
    status 2:
            fix yaw
            make x = 0
            make z = 0
            make y = 2
            get center
            set status according to center

if y < 2.5m:
    status 3:
            fix yaw
            make x = 0
            make z = 0
            get center
            fly to the center.

}
