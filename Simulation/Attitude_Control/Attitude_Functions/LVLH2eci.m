function [eci_vec] = LVLH2eci(LVLH_vec,RAAN,i,theta)
LVLH2eci = [-sin(RAAN)*cos(i)*sin(theta)+cos(RAAN)*cos(theta),-sin(RAAN)*cos(i)*cos(theta)-cos(RAAN)*sin(theta),sin(i)*sin(RAAN);...
            cos(RAAN)*cos(i)*sin(theta) + sin(RAAN)*cos(theta) , cos(RAAN)*cos(i)*cos(theta) - sin(RAAN)*sin(theta), -sin(i)*cos(RAAN);...
            sin(theta)*sin(i), cos(theta)*sin(i), cos(i)];

eci_vec = LVLH2eci*LVLH_vec;
