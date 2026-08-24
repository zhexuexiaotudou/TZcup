// TZCup formal sanitation vehicle packaging model (millimetres).
// Project-designed parametric assembly; dynamics live in the matching Xacro.
$fn = 48;
$explode = 0;

module rounded_box(size, r=4) {
  minkowski() {
    cube([size[0]-2*r, size[1]-2*r, size[2]-2*r], center=true);
    sphere(r=r);
  }
}

module wheel() {
  color("#202428") rotate([90,0,0]) cylinder(h=88, r=165, center=true);
  color("#b7bec5") rotate([90,0,0]) cylinder(h=92, r=58, center=true);
}

module a300_platform() {
  color("#343a40") translate([0,0,225]) rounded_box([790,570,190], 18);
  color("#111820") translate([0,0,345]) rounded_box([650,500,70], 10);
  for (x=[-275,275], y=[-330,330]) translate([x,y,165]) wheel();
  color("#6c757d") translate([0,0,410]) cube([720,520,20], center=true);
}

module side_brush(x,y) {
  color("#444") translate([x,y,78]) cylinder(h=38, r=46, center=true);
  color("#f5a623") translate([x,y,54]) cylinder(h=12, r=155, center=true);
  for (a=[0:30:330]) color("#d8a13a")
    translate([x,y,48]) rotate([0,0,a]) translate([92,0,0]) cube([150,8,5], center=true);
}

module cleaning_head() {
  color("#59636e") translate([245,0,105]) rounded_box([190,500,80], 12);
  side_brush(290,-260); side_brush(290,260);
  color("#2b2b2b") translate([145,0,56]) rotate([90,0,0]) cylinder(h=500,r=55,center=true);
  color("#1565c0") translate([-330,0,60]) cube([35,550,65], center=true);
  color("#111") translate([-360,0,32]) cube([15,610,15], center=true);
  color("#2f3b45") translate([-300,0,145]) rounded_box([210,460,160], 18);
}

module storage() {
  color([0.13,0.36,0.23,0.85]) translate([-120,95,660+$explode]) rounded_box([430,330,500], 18);
  color([0.1,0.38,0.63,0.82]) translate([-165,-180,580+$explode]) rounded_box([330,180,300], 18);
  color("#cdd6dd") translate([-165,-180,735+$explode]) cube([310,160,16], center=true);
  color("#cfd8dc") translate([-250,-260,440]) rotate([0,90,0]) cylinder(h=180,r=42,center=true);
}

module sensor_mast() {
  color("#8b939a") translate([80,0,735+$explode]) cube([65,65,610],center=true);
  color("#101820") translate([80,0,1045+2*$explode]) cylinder(h=70,r=40,center=true);
  color("#212121") translate([150,0,890+2*$explode]) rounded_box([105,55,36], 5);
  color("#222") translate([95,-320,710+$explode]) rounded_box([32,46,36], 5);
  color("#222") translate([95,320,710+$explode]) rounded_box([32,46,36], 5);
  color("#eceff1") translate([25,0,1120+2*$explode]) rounded_box([60,60,35], 5);
}

module arm_segment(p1,p2,r=55) {
  v = p2-p1; len=norm(v);
  translate(p1) rotate([0,acos(v[2]/len),atan2(v[1],v[0])])
    translate([0,0,len/2]) cylinder(h=len,r=r,center=true);
}

module ur5e_arm() {
  color("#98a4ae") translate([80,-170,520+$explode]) cylinder(h=170,r=92,center=true);
  color("#7a8791") {
    arm_segment([80,-170,605+$explode],[80,-170,1030+2*$explode],62);
    arm_segment([80,-170,1030+2*$explode],[430,-170,1160+3*$explode],55);
    arm_segment([430,-170,1160+3*$explode],[620,-170,980+4*$explode],46);
  }
  color("#263238") translate([620,-170,980+4*$explode]) rotate([0,90,0]) cylinder(h=105,r=55,center=true);
  color("#424b50") translate([705,-170,980+4*$explode]) rounded_box([95,105,95], 8);
  color("#1f2529") for (y=[-55,55]) translate([765,-170+y,980+4*$explode]) cube([90,18,32],center=true);
}

module electronics_cabinet() {
  color([0.16,0.19,0.22,0.9]) translate([-75,-60,520]) rounded_box([470,170,180], 12);
  color("#2a7fff") translate([-75,-150,520]) cube([250,8,80],center=true);
}

module vehicle() {
  a300_platform();
  cleaning_head();
  storage();
  electronics_cabinet();
  sensor_mast();
  ur5e_arm();
}

vehicle();
