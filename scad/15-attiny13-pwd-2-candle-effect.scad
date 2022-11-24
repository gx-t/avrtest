d=30.1;
h=7.85;
t=2;

module base() {
    difference() {
        cylinder(h=h*1.25+t, r=d/2+t, $fn=16, center=true);
        cylinder(h=h*1.25, r=d/2, $fn=256, center=true);
        cube([d*2,d*2,h/4], center=true);
    }
}

module latch_1(z) {
    for(a=[0:90:270]) {
        rotate([0, 0, a]) {
            translate([d/2+t/2, 0, z]) {
                sphere(r=t/3, $fn=32);
            }
        }
    }
}


difference() {
    union() {
        base();
        latch_1(h/8-t/10);
    }
    latch_1(-h/8-t/10);
}
