function q = euler_to_quaternion(roll, pitch, yaw)
  cr = cos(deg2rad(roll/2));
  cp = cos(deg2rad(pitch/2));
  cy = cos(deg2rad(yaw/2));
  sr = sin(deg2rad(roll/2));
  sp = sin(deg2rad(pitch/2));
  sy = sin(deg2rad(yaw/2));
  q = [sr*cp*cy-cr*sp*sy;       cr*sp*cy+sr*cp*sy;       cr*cp*sy-sr*sp*cy;      cr*cp*cy+sr*sp*sy];
end