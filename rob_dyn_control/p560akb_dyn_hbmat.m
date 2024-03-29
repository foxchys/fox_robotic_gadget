%     This file is part of fox_robotic_gadget/rob_dyn_control.
%     
%     Copyright (C) 2021-2022, by ChyS(foxchys)
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Affero General Public License as published
%     by the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Affero General Public License for more details.
% 
%     You should have received a copy of the GNU Affero General Public License
%     along with this program.  If not, see <https://www.gnu.org/licenses/>.
%     
%     ChyS(foxchys): https://github.com/foxchys

function [dyn_hbmat] = p560akb_dyn_hbmat(q, dq, ddq)
% compute torque of joints based on 
%       theoretically dynamic params      
%  input:
%   q： sequential joint position
%             e.g. [joint1_t1, joint2_t1,....jointn_t1;
%                   joint1_t2, joint2_t2,,....jointn_t2;
%                   ..................................;
%                   joint1_tn, joint2_tn,....jointn_tn]
%   dq： sequential joint velocity
%   ddq： sequential joint acceleration
%  output:
%   dyn_hbmat: dynamic base linear regressor matrix


%;
    dyn_hbmat =zeros(48, 6);
%;


    x0 = sin(q(2));
    x1 = -1.0*dq(1);
    x2 = x0*x1;
    x3 = cos(q(2));
    x4 = x1*x3;
    x5 = x2*x4;
    x6 = -x5;
    x7 = 6.12323399573677e-17*dq(1) + dq(2);
    x8 = x2*x7;
    x9 = -1.0*ddq(1);
    x10 = dq(2)*x4 + x0*x9;
    x11 = 1.0*dq(1)*dq(2)*x0 + x3*x9;
    x12 = -x8;
    x13 = x4*x7;
    x14 = x2^2;
    x15 = x4^2;
    x16 = -x15;
    x17 = x14 + x16;
    x18 = x10 - x13;
    x19 = -x14;
    x20 = x7^2;
    x21 = 6.12323399573677e-17*ddq(1) + ddq(2);
    x22 = x21 + x5;
    x23 = x0*x22;
    x24 = -x20;
    x25 = x11 + x8;
    x26 = x16 + x24;
    x27 = -dq(1)^2;
    x28 = 0.2435*x27 + 6.00689254981777e-16;
    x29 = 1.4910074779619e-17*x27;
    x30 = -ddq(1);
    x31 = -0.2435*x0*x30 + x29*x3 - 9.81*x3;
    x32 = x0*x29 - 9.81*x0 + 0.2435*x3*x30;
    x33 = -x32;
    x34 = -1.0*x0;
    x35 = cos(q(3));
    x36 = sin(q(3));
    x37 = -x36;
    x38 = x2*x37 + x35*x4;
    x39 = dq(3)*x38 + x10*x35 + x11*x36;
    x40 = x2*x35 + x36*x4;
    x41 = dq(3) + x7;
    x42 = x40*x41;
    x43 = x38*x40;
    x44 = -x43;
    x45 = -x42;
    x46 = -x10;
    x47 = -x40;
    x48 = dq(3)*x47 + x11*x35 + x36*x46;
    x49 = x45 + x48;
    x50 = x38*x41;
    x51 = x39 + x50;
    x52 = x38^2;
    x53 = -x52;
    x54 = x40^2;
    x55 = x53 + x54;
    x56 = x39 - x50;
    x57 = ddq(3) + x21;
    x58 = x43 + x57;
    x59 = x35*x58;
    x60 = x41^2;
    x61 = -x54;
    x62 = x60 + x61;
    x63 = x36*x58;
    x64 = x42 + x48;
    x65 = -x60;
    x66 = x52 + x65;
    x67 = x44 + x57;
    x68 = 6.12323399573677e-17*x57;
    x69 = -0.4318*x11 - 0.0934*x16 - 0.0934*x19 + x28 + 0.4318*x8;
    x70 = -x69;
    x71 = x53 + x65;
    x72 = x35*x71;
    x73 = x42 - x48;
    x74 = x36*x71;
    x75 = -0.0934*x13 + 0.4318*x22 + x31 - 0.0934*x46;
    x76 = -0.0934*x25 + 0.4318*x26 + x32;
    x77 = x35*x75 + x37*x76;
    x78 = 0.4318*x59 + 0.4318*x74 + x77;
    x79 = x61 + x65;
    x80 = x35*x79;
    x81 = x43 - x57;
    x82 = x36*x81;
    x83 = x35*x76 + x36*x75;
    x84 = -x83;
    x85 = 0.4318*x80 + 0.4318*x82 + x84;
    x86 = x35*x81;
    x87 = 0.0934*x36;
    x88 = cos(q(4));
    x89 = sin(q(4));
    x90 = 6.12323399573677e-17*x38;
    x91 = 1.0*x41;
    x92 = x40*x88 + x89*x90 + x89*x91;
    x93 = dq(4) - 1.0*x38 + 6.12323399573677e-17*x41;
    x94 = x92*x93;
    x95 = x88*x94;
    x96 = 6.12323399573677e-17*x48;
    x97 = x47*x89 + x88*x90 + x88*x91;
    x98 = 1.0*x57;
    x99 = dq(4)*x97 + x39*x88 + x89*x96 + x89*x98;
    x100 = x89*x99;
    x101 = x92*x97;
    x102 = -x101;
    x103 = 6.12323399573677e-17*x100 - 1.0*x102 + 6.12323399573677e-17*x95;
    x104 = -x89;
    x105 = x104*x94 + x88*x99;
    x106 = 1.0*x100 + 6.12323399573677e-17*x102 + 1.0*x95;
    x107 = x93*x97;
    x108 = x107 + x99;
    x109 = x108*x88;
    x110 = x97^2;
    x111 = -x110;
    x112 = x92^2;
    x113 = x111 + x112;
    x114 = -x94;
    x115 = -x92;
    x116 = -x39;
    x117 = dq(4)*x115 + x116*x89 + x88*x96 + x88*x98;
    x118 = x114 + x117;
    x119 = x118*x89;
    x120 = 6.12323399573677e-17*x109 - 1.0*x113 + 6.12323399573677e-17*x119;
    x121 = x104*x108 + x118*x88;
    x122 = 1.0*x109 + 6.12323399573677e-17*x113 + 1.0*x119;
    x123 = x93^2;
    x124 = -x112;
    x125 = x123 + x124;
    x126 = x125*x88;
    x127 = -x107 + x99;
    x128 = ddq(4) - 1.0*x48 + x68;
    x129 = x101 + x128;
    x130 = x129*x89;
    x131 = 1.0*x126 + 6.12323399573677e-17*x127 + 1.0*x130;
    x132 = x129*x88;
    x133 = x104*x125 + x132;
    x134 = 6.12323399573677e-17*x126 - 1.0*x127 + 6.12323399573677e-17*x130;
    x135 = x102 + x128;
    x136 = x135*x88;
    x137 = -x123;
    x138 = x110 + x137;
    x139 = x138*x89;
    x140 = x117 + x94;
    x141 = 1.0*x136 + 1.0*x139 + 6.12323399573677e-17*x140;
    x142 = 6.12323399573677e-17*x136 + 6.12323399573677e-17*x139 - 1.0*x140;
    x143 = x104*x135 + x138*x88;
    x144 = x104*x114 + x107*x88;
    x145 = x114*x88;
    x146 = -1.0*x128;
    x147 = x107*x89;
    x148 = 6.12323399573677e-17*x145 + x146 + 6.12323399573677e-17*x147;
    x149 = 6.12323399573677e-17*x128;
    x150 = 1.0*x145 + 1.0*x147 + x149;
    x151 = -x117 + x94;
    x152 = x111 + x137;
    x153 = x152*x89;
    x154 = 6.12323399573677e-17*x132 - 1.0*x151 + 6.12323399573677e-17*x153;
    x155 = x152*x88;
    x156 = x104*x129 + x155;
    x157 = x156*x35;
    x158 = x116 + x50;
    x159 = x53 + x61;
    x160 = -2.65197264355359e-17*x158 + 1.62386490467713e-33*x159 - 2.65197264355359e-17*x51 + 0.0203*x58 + 6.12323399573677e-17*x69 - 1.24301650113456e-18*x73 - 1.0*x77 + 0.4331*x79;
    x161 = -x160;
    x162 = x161*x88;
    x163 = 2.65197264355359e-17*x64 - 0.0203*x71 - 0.4331*x81 + x83;
    x164 = 2.65197264355359e-17*x159 - 0.4331*x51 + 1.0*x69 - 0.0203*x73;
    x165 = 1.62386490467713e-33*x158 - 1.24301650113456e-18*x58 + 6.12323399573677e-17*x77 - 2.65197264355359e-17*x79;
    x166 = x104*x163 + x164*x88 + x165*x88;
    x167 = -2.65197264355359e-17*x130 + 0.0203*x132 + 1.24301650113456e-18*x151 + 0.0203*x153 + 2.65197264355359e-17*x155 + 6.12323399573677e-17*x162 - 1.0*x166;
    x168 = x154*x35;
    x169 = x104*x161 - 0.4331*x132 - 0.4331*x153;
    x170 = 6.12323399573677e-17*x166;
    x171 = -0.4331*x130 - 1.24301650113456e-18*x132 + 0.0203*x151 - 1.24301650113456e-18*x153 + 0.4331*x155 + 1.0*x162 + x170;
    x172 = x156*x36;
    x173 = 0.4318*x168 + x171 + 0.4318*x172;
    x174 = x101 - x128;
    x175 = x174*x89;
    x176 = x124 + x137;
    x177 = x176*x88;
    x178 = x160*x88 - 0.4331*x175 - 0.4331*x177;
    x179 = -1.0*x108 + 6.12323399573677e-17*x175 + 6.12323399573677e-17*x177;
    x180 = x179*x35;
    x181 = x174*x88;
    x182 = x176*x89;
    x183 = x163*x88 + x164*x89 + x165*x89;
    x184 = -x183;
    x185 = 6.12323399573677e-17*x160;
    x186 = 1.24301650113456e-18*x108 + 0.0203*x175 + 0.0203*x177 + 2.65197264355359e-17*x181 - 2.65197264355359e-17*x182 - 1.0*x184 + x185*x89;
    x187 = x104*x176 + x181;
    x188 = x187*x35;
    x189 = x187*x36;
    x190 = 1.0*x89;
    x191 = 0.0203*x108 + x160*x190 - 1.24301650113456e-18*x175 - 1.24301650113456e-18*x177 + 0.4331*x181 - 0.4331*x182 + 6.12323399573677e-17*x184;
    x192 = 0.4318*x180 + 0.4318*x189 + x191;
    x193 = sin(q(5));
    x194 = 6.12323399573677e-17*x97;
    x195 = -1.0*x93;
    x196 = cos(q(5));
    x197 = x193*x194 + x193*x195 + x196*x92;
    x198 = x115*x193 + x194*x196 + x195*x196;
    x199 = x197*x198;
    x200 = -x199;
    x201 = dq(5) + 6.12323399573677e-17*x93 + 1.0*x97;
    x202 = x197*x201;
    x203 = x196*x202;
    x204 = 6.12323399573677e-17*x117;
    x205 = dq(5)*x198 + x146*x193 + x193*x204 + x196*x99;
    x206 = x193*x205;
    x207 = 1.0*x200 + 6.12323399573677e-17*x203 + 6.12323399573677e-17*x206;
    x208 = -x193;
    x209 = x196*x205 + x202*x208;
    x210 = x104*x207 + x209*x88;
    x211 = 6.12323399573677e-17*x200 - 1.0*x203 - 1.0*x206;
    x212 = x207*x88;
    x213 = 6.12323399573677e-17*x89;
    x214 = x209*x213 - 1.0*x211 + 6.12323399573677e-17*x212;
    x215 = x190*x209 + 6.12323399573677e-17*x211 + 1.0*x212;
    x216 = x198*x201;
    x217 = x205 + x216;
    x218 = -x197;
    x219 = dq(5)*x218 + x146*x196 - x193*x99 + x196*x204;
    x220 = -x202;
    x221 = x219 + x220;
    x222 = x196*x221 + x208*x217;
    x223 = x196*x217;
    x224 = x197^2;
    x225 = x198^2;
    x226 = -x225;
    x227 = x224 + x226;
    x228 = x193*x221;
    x229 = -1.0*x223 + 6.12323399573677e-17*x227 - 1.0*x228;
    x230 = 6.12323399573677e-17*x223 + 1.0*x227 + 6.12323399573677e-17*x228;
    x231 = x230*x88;
    x232 = x190*x222 + 6.12323399573677e-17*x229 + 1.0*x231;
    x233 = x104*x230 + x222*x88;
    x234 = x213*x222 - 1.0*x229 + 6.12323399573677e-17*x231;
    x235 = x201^2;
    x236 = -x224;
    x237 = x235 + x236;
    x238 = x196*x237;
    x239 = x205 - x216;
    x240 = ddq(5) + 1.0*x117 + x149;
    x241 = x199 + x240;
    x242 = x193*x241;
    x243 = 6.12323399573677e-17*x238 + 1.0*x239 + 6.12323399573677e-17*x242;
    x244 = x196*x241;
    x245 = x208*x237 + x244;
    x246 = x104*x243 + x245*x88;
    x247 = x243*x88;
    x248 = -1.0*x238 + 6.12323399573677e-17*x239 - 1.0*x242;
    x249 = x213*x245 + 6.12323399573677e-17*x247 - 1.0*x248;
    x250 = x190*x245 + 1.0*x247 + 6.12323399573677e-17*x248;
    x251 = -x235;
    x252 = x225 + x251;
    x253 = x200 + x240;
    x254 = x196*x252 + x208*x253;
    x255 = x193*x252;
    x256 = x196*x253;
    x257 = x202 + x219;
    x258 = 6.12323399573677e-17*x255 + 6.12323399573677e-17*x256 + 1.0*x257;
    x259 = x258*x88;
    x260 = -1.0*x255 - 1.0*x256 + 6.12323399573677e-17*x257;
    x261 = x213*x254 + 6.12323399573677e-17*x259 - 1.0*x260;
    x262 = x104*x258 + x254*x88;
    x263 = x190*x254 + 1.0*x259 + 6.12323399573677e-17*x260;
    x264 = x196*x216 + x208*x220;
    x265 = x196*x220;
    x266 = 6.12323399573677e-17*x240;
    x267 = x193*x216;
    x268 = -1.0*x265 + x266 - 1.0*x267;
    x269 = 1.0*x240;
    x270 = 6.12323399573677e-17*x265 + 6.12323399573677e-17*x267 + x269;
    x271 = x270*x88;
    x272 = x213*x264 - 1.0*x268 + 6.12323399573677e-17*x271;
    x273 = x104*x270 + x264*x88;
    x274 = x190*x264 + 6.12323399573677e-17*x268 + 1.0*x271;
    x275 = x202 - x219;
    x276 = x226 + x251;
    x277 = x193*x276;
    x278 = 6.12323399573677e-17*x244 + 1.0*x275 + 6.12323399573677e-17*x277;
    x279 = x278*x88;
    x280 = 1.0*x166 + x185;
    x281 = -x280;
    x282 = x196*x281;
    x283 = -1.0*x160;
    x284 = x170*x196 + x184*x193 + x196*x283;
    x285 = 6.12323399573677e-17*x282 + 1.0*x284;
    x286 = x196*x276 + x208*x241;
    x287 = x286*x89;
    x288 = x208*x281;
    x289 = x104*x285 - 0.4331*x279 - 0.4331*x287 + x288*x88;
    x290 = -1.0*x244 + 6.12323399573677e-17*x275 - 1.0*x277;
    x291 = x286*x88;
    x292 = x278*x89;
    x293 = 6.12323399573677e-17*x284;
    x294 = -1.0*x282 + x293;
    x295 = x285*x88;
    x296 = x213*x288 + 0.0203*x279 + 0.0203*x287 + 1.24301650113456e-18*x290 + 2.65197264355359e-17*x291 - 2.65197264355359e-17*x292 - 1.0*x294 + 6.12323399573677e-17*x295;
    x297 = x104*x278 + x291;
    x298 = x297*x35;
    x299 = x213*x286 + 6.12323399573677e-17*x279 - 1.0*x290;
    x300 = x299*x35;
    x301 = x190*x288 - 1.24301650113456e-18*x279 - 1.24301650113456e-18*x287 + 0.0203*x290 + 0.4331*x291 - 0.4331*x292 + 6.12323399573677e-17*x294 + 1.0*x295;
    x302 = x297*x36;
    x303 = 0.4318*x300 + x301 + 0.4318*x302;
    x304 = x236 + x251;
    x305 = x199 - x240;
    x306 = x196*x305 + x208*x304;
    x307 = x306*x89;
    x308 = x196*x304;
    x309 = x193*x305;
    x310 = 1.0*x217 + 6.12323399573677e-17*x308 + 6.12323399573677e-17*x309;
    x311 = x310*x88;
    x312 = x170*x193 + x183*x196 + x193*x283;
    x313 = -x312;
    x314 = -1.0*x193;
    x315 = x280*x314 + 6.12323399573677e-17*x313;
    x316 = x310*x89;
    x317 = x196*x280;
    x318 = 6.12323399573677e-17*x280;
    x319 = x193*x318 + 1.0*x313;
    x320 = x319*x88;
    x321 = x306*x88;
    x322 = 6.12323399573677e-17*x217 - 1.0*x308 - 1.0*x309;
    x323 = x213*x317 + 0.0203*x307 + 0.0203*x311 - 1.0*x315 - 2.65197264355359e-17*x316 + 6.12323399573677e-17*x320 + 2.65197264355359e-17*x321 + 1.24301650113456e-18*x322;
    x324 = x213*x306 + 6.12323399573677e-17*x311 - 1.0*x322;
    x325 = x324*x35;
    x326 = x104*x310 + x321;
    x327 = x104*x319 - 0.4331*x307 - 0.4331*x311 + x317*x88;
    x328 = x326*x36;
    x329 = x326*x35;
    x330 = x190*x317 - 1.24301650113456e-18*x307 - 1.24301650113456e-18*x311 + 6.12323399573677e-17*x315 - 0.4331*x316 + 1.0*x320 + 0.4331*x321 + 0.0203*x322;
    x331 = 0.4318*x325 + 0.4318*x328 + x330;
    x332 = cos(q(6));
    x333 = sin(q(6));
    x334 = 1.0*x201;
    x335 = 6.12323399573677e-17*x198;
    x336 = x197*x332 + x333*x334 + x333*x335;
    x337 = dq(6) - 1.0*x198 + 6.12323399573677e-17*x201;
    x338 = x336*x337;
    x339 = x332*x338;
    x340 = x218*x333 + x332*x334 + x332*x335;
    x341 = 6.12323399573677e-17*x219;
    x342 = dq(6)*x340 + x205*x332 + x269*x333 + x333*x341;
    x343 = x333*x342;
    x344 = x336*x340;
    x345 = -x344;
    x346 = 6.12323399573677e-17*x339 + 6.12323399573677e-17*x343 - 1.0*x345;
    x347 = x196*x346;
    x348 = 1.0*x339 + 1.0*x343 + 6.12323399573677e-17*x345;
    x349 = -x333;
    x350 = x332*x342 + x338*x349;
    x351 = x314*x350 - 1.0*x347 + 6.12323399573677e-17*x348;
    x352 = x196*x350 + x208*x346;
    x353 = 6.12323399573677e-17*x193;
    x354 = 6.12323399573677e-17*x347 + 1.0*x348 + x350*x353;
    x355 = x354*x88;
    x356 = x213*x352 - 1.0*x351 + 6.12323399573677e-17*x355;
    x357 = x104*x354 + x352*x88;
    x358 = x190*x352 + 6.12323399573677e-17*x351 + 1.0*x355;
    x359 = -x338;
    x360 = -dq(6)*x336 - x205*x333 + x269*x332 + x332*x341;
    x361 = x359 + x360;
    x362 = x333*x361;
    x363 = x340^2;
    x364 = -x363;
    x365 = x336^2;
    x366 = x364 + x365;
    x367 = x337*x340;
    x368 = x342 + x367;
    x369 = x332*x368;
    x370 = 6.12323399573677e-17*x362 - 1.0*x366 + 6.12323399573677e-17*x369;
    x371 = x332*x361 + x349*x368;
    x372 = x196*x371 + x208*x370;
    x373 = x196*x370;
    x374 = 1.0*x362 + 6.12323399573677e-17*x366 + 1.0*x369;
    x375 = x353*x371 + 6.12323399573677e-17*x373 + 1.0*x374;
    x376 = x375*x88;
    x377 = x314*x371 - 1.0*x373 + 6.12323399573677e-17*x374;
    x378 = x190*x372 + 1.0*x376 + 6.12323399573677e-17*x377;
    x379 = x213*x372 + 6.12323399573677e-17*x376 - 1.0*x377;
    x380 = x104*x375 + x372*x88;
    x381 = x337^2;
    x382 = -x365;
    x383 = x381 + x382;
    x384 = x332*x383;
    x385 = ddq(6) - 1.0*x219 + x266;
    x386 = x344 + x385;
    x387 = x333*x386;
    x388 = x342 - x367;
    x389 = 1.0*x384 + 1.0*x387 + 6.12323399573677e-17*x388;
    x390 = x332*x386;
    x391 = x349*x383 + x390;
    x392 = 6.12323399573677e-17*x384 + 6.12323399573677e-17*x387 - 1.0*x388;
    x393 = x196*x392;
    x394 = x353*x391 + 1.0*x389 + 6.12323399573677e-17*x393;
    x395 = x394*x88;
    x396 = x196*x391 + x208*x392;
    x397 = x314*x391 + 6.12323399573677e-17*x389 - 1.0*x393;
    x398 = x213*x396 + 6.12323399573677e-17*x395 - 1.0*x397;
    x399 = x104*x394 + x396*x88;
    x400 = x190*x396 + 1.0*x395 + 6.12323399573677e-17*x397;
    x401 = x345 + x385;
    x402 = -x381;
    x403 = x363 + x402;
    x404 = x332*x403 + x349*x401;
    x405 = x338 + x360;
    x406 = x333*x403;
    x407 = x332*x401;
    x408 = -1.0*x405 + 6.12323399573677e-17*x406 + 6.12323399573677e-17*x407;
    x409 = x196*x404 + x208*x408;
    x410 = 6.12323399573677e-17*x405 + 1.0*x406 + 1.0*x407;
    x411 = x196*x408;
    x412 = x314*x404 + 6.12323399573677e-17*x410 - 1.0*x411;
    x413 = x353*x404 + 1.0*x410 + 6.12323399573677e-17*x411;
    x414 = x413*x88;
    x415 = x190*x409 + 6.12323399573677e-17*x412 + 1.0*x414;
    x416 = x104*x413 + x409*x88;
    x417 = x213*x409 - 1.0*x412 + 6.12323399573677e-17*x414;
    x418 = x332*x359;
    x419 = x333*x367;
    x420 = -1.0*x385 + 6.12323399573677e-17*x418 + 6.12323399573677e-17*x419;
    x421 = x332*x367 + x349*x359;
    x422 = x196*x421 + x208*x420;
    x423 = x196*x420;
    x424 = 6.12323399573677e-17*x385 + 1.0*x418 + 1.0*x419;
    x425 = x353*x421 + 6.12323399573677e-17*x423 + 1.0*x424;
    x426 = x104*x425 + x422*x88;
    x427 = x425*x88;
    x428 = x314*x421 - 1.0*x423 + 6.12323399573677e-17*x424;
    x429 = x213*x422 + 6.12323399573677e-17*x427 - 1.0*x428;
    x430 = x190*x422 + 1.0*x427 + 6.12323399573677e-17*x428;
    x431 = x338 - x360;
    x432 = x364 + x402;
    x433 = x333*x432;
    x434 = 6.12323399573677e-17*x390 - 1.0*x431 + 6.12323399573677e-17*x433;
    x435 = x332*x432 + x349*x386;
    x436 = x196*x435 + x208*x434;
    x437 = x436*x88;
    x438 = 1.0*x390 + 6.12323399573677e-17*x431 + 1.0*x433;
    x439 = x196*x434;
    x440 = x353*x435 + 1.0*x438 + 6.12323399573677e-17*x439;
    x441 = x104*x440 + x437;
    x442 = x36*x441;
    x443 = x440*x88;
    x444 = x314*x435 + 6.12323399573677e-17*x438 - 1.0*x439;
    x445 = x213*x436 + 6.12323399573677e-17*x443 - 1.0*x444;
    x446 = x35*x445;
    x447 = x436*x89;
    x448 = 1.0*x280;
    x449 = x293*x332 + x313*x333 + x332*x448;
    x450 = -1.0*x284 + x318;
    x451 = -x450;
    x452 = x332*x451;
    x453 = -1.0*x449 + 6.12323399573677e-17*x452;
    x454 = x196*x453;
    x455 = 6.12323399573677e-17*x449 + 1.0*x452;
    x456 = x349*x451;
    x457 = x353*x456 + 6.12323399573677e-17*x454 + 1.0*x455;
    x458 = x457*x88;
    x459 = x314*x456 - 1.0*x454 + 6.12323399573677e-17*x455;
    x460 = x196*x456 + x208*x453;
    x461 = x440*x89;
    x462 = x213*x460 + 2.65197264355359e-17*x437 + 0.0203*x443 + 1.24301650113456e-18*x444 + 0.0203*x447 + 6.12323399573677e-17*x458 - 1.0*x459 - 2.65197264355359e-17*x461;
    x463 = x35*x441;
    x464 = x104*x457 - 0.4331*x443 - 0.4331*x447 + x460*x88;
    x465 = x190*x460 + 0.4331*x437 - 1.24301650113456e-18*x443 + 0.0203*x444 - 1.24301650113456e-18*x447 + 1.0*x458 + 6.12323399573677e-17*x459 - 0.4331*x461;
    x466 = 0.4318*x442 + 0.4318*x446 + x465;
    x467 = x344 - x385;
    x468 = x382 + x402;
    x469 = x332*x467 + x349*x468;
    x470 = x333*x467;
    x471 = x332*x468;
    x472 = -1.0*x368 + 6.12323399573677e-17*x470 + 6.12323399573677e-17*x471;
    x473 = x196*x469 + x208*x472;
    x474 = 6.12323399573677e-17*x368 + 1.0*x470 + 1.0*x471;
    x475 = x196*x472;
    x476 = x353*x469 + 1.0*x474 + 6.12323399573677e-17*x475;
    x477 = x476*x88;
    x478 = x314*x469 + 6.12323399573677e-17*x474 - 1.0*x475;
    x479 = x213*x473 + 6.12323399573677e-17*x477 - 1.0*x478;
    x480 = x35*x479;
    x481 = x473*x88;
    x482 = x104*x476 + x481;
    x483 = x36*x482;
    x484 = x476*x89;
    x485 = x332*x450;
    x486 = -x293*x333 - x312*x332 - x333*x448;
    x487 = x333*x450;
    x488 = 6.12323399573677e-17*x486 + 1.0*x487;
    x489 = -1.0*x486 + 6.12323399573677e-17*x487;
    x490 = x196*x489;
    x491 = x314*x485 + 6.12323399573677e-17*x488 - 1.0*x490;
    x492 = x196*x485 + x208*x489;
    x493 = x353*x485 + 1.0*x488 + 6.12323399573677e-17*x490;
    x494 = x493*x88;
    x495 = x473*x89;
    x496 = x213*x492 + 0.0203*x477 + 1.24301650113456e-18*x478 + 2.65197264355359e-17*x481 - 2.65197264355359e-17*x484 - 1.0*x491 + 6.12323399573677e-17*x494 + 0.0203*x495;
    x497 = x104*x493 - 0.4331*x477 + x492*x88 - 0.4331*x495;
    x498 = x35*x482;
    x499 = x190*x492 - 1.24301650113456e-18*x477 + 0.0203*x478 + 0.4331*x481 - 0.4331*x484 + 6.12323399573677e-17*x491 + 1.0*x494 - 1.24301650113456e-18*x495;
    x500 = 0.4318*x480 + 0.4318*x483 + x499;
%;
    dyn_hbmat(1) = ddq(1);
    dyn_hbmat(2) = dq(1);
    dyn_hbmat(3) = sign(dq(1));
    dyn_hbmat(4) = -1.0*x0*x10 - 1.0*x3*x8 + 6.12323399573677e-17*x6;
    dyn_hbmat(5) = -1.0*x0*(x11 + x12) + 6.12323399573677e-17*x17 - 1.0*x3*(x10 + x13);
    dyn_hbmat(6) = 6.12323399573677e-17*x18 - 1.0*x23 - 1.0*x3*(x19 + x20);
    dyn_hbmat(7) = -1.0*x0*(x15 + x24) + 6.12323399573677e-17*x25 - 1.0*x3*(x21 + x6);
    dyn_hbmat(8) = -1.0*x0*x13 - 1.0*x12*x3 + 6.12323399573677e-17*x21;
    dyn_hbmat(9) = 0.2435*x23 - 0.2435*x26*x3 + 1.0*x28*x3 + 6.12323399573677e-17*x31;
    dyn_hbmat(10) = -1.0*x0*x28 + 0.2435*x0*(x19 + x24) - 0.2435*x3*(-x21 + x5) + 6.12323399573677e-17*x33;
    dyn_hbmat(11) = 0;
    dyn_hbmat(12) = 0;
    dyn_hbmat(13) = -1.0*x3*(x35*x42 + x36*x39) + x34*(x35*x39 + x37*x42) + 6.12323399573677e-17*x44;
    dyn_hbmat(14) = -1.0*x3*(x35*x51 + x36*x49) + x34*(x35*x49 + x37*x51) + 6.12323399573677e-17*x55;
    dyn_hbmat(15) = -1.0*x3*(x35*x62 + x63) + x34*(x37*x62 + x59) + 6.12323399573677e-17*x56;
    dyn_hbmat(16) = -1.0*x3*(x35*x67 + x36*x66) + x34*(x35*x66 + x37*x67) + 6.12323399573677e-17*x64;
    dyn_hbmat(17) = -1.0*x3*(x35*x45 + x36*x50) + x34*(x35*x50 + x37*x45) + x68;
    dyn_hbmat(18) = 0.2435*x0*(x59 + x74) - 0.2435*x3*(x37*x58 + x72) - 1.0*x3*(x35*x70 + 0.0934*x63 - 0.0934*x72 - 0.4318*x73) + x34*(x37*x70 + 0.0934*x59 + 0.0934*x74) + 6.12323399573677e-17*x78;
    dyn_hbmat(19) = 0.2435*x0*(x80 + x82) - 0.2435*x3*(x37*x79 + x86) - 1.0*x3*(x36*x69 - 0.4318*x51 + x79*x87 - 0.0934*x86) + x34*(x35*x69 + 0.0934*x80 + 0.0934*x82) + 6.12323399573677e-17*x85;
    dyn_hbmat(20) = 0;
    dyn_hbmat(21) = 0;
    dyn_hbmat(22) = 6.12323399573677e-17*x106 - 1.0*x3*(x103*x35 + x105*x36) + x34*(x103*x37 + x105*x35);
    dyn_hbmat(23) = 6.12323399573677e-17*x122 - 1.0*x3*(x120*x35 + x121*x36) + x34*(x120*x37 + x121*x35);
    dyn_hbmat(24) = 6.12323399573677e-17*x131 - 1.0*x3*(x133*x36 + x134*x35) + x34*(x133*x35 + x134*x37);
    dyn_hbmat(25) = 6.12323399573677e-17*x141 - 1.0*x3*(x142*x35 + x143*x36) + x34*(x142*x37 + x143*x35);
    dyn_hbmat(26) = 6.12323399573677e-17*x150 - 1.0*x3*(x144*x36 + x148*x35) + x34*(x144*x35 + x148*x37);
    dyn_hbmat(27) = 0.2435*x0*(x168 + x172) + 6.12323399573677e-17*x173 - 0.2435*x3*(x154*x37 + x157) - 1.0*x3*(-0.4318*x132 - 2.64401243935914e-17*x151 - 0.4318*x153 + x154*x87 - 0.0934*x157 + x167*x35 + x169*x36) + x34*(x156*x87 + x167*x37 + 0.0934*x168 + x169*x35);
    dyn_hbmat(28) = 0.2435*x0*(x180 + x189) + 6.12323399573677e-17*x192 - 0.2435*x3*(x179*x37 + x188) - 1.0*x3*(-2.64401243935914e-17*x108 - 0.4318*x175 - 0.4318*x177 + x178*x36 + x179*x87 + x186*x35 - 0.0934*x188) + x34*(x178*x35 + 0.0934*x180 + x186*x37 + x187*x87);
    dyn_hbmat(29) = 0;
    dyn_hbmat(30) = 0;
    dyn_hbmat(31) = 6.12323399573677e-17*x215 - 1.0*x3*(x210*x36 + x214*x35) + x34*(x210*x35 + x214*x37);
    dyn_hbmat(32) = 6.12323399573677e-17*x232 - 1.0*x3*(x233*x36 + x234*x35) + x34*(x233*x35 + x234*x37);
    dyn_hbmat(33) = 6.12323399573677e-17*x250 - 1.0*x3*(x246*x36 + x249*x35) + x34*(x246*x35 + x249*x37);
    dyn_hbmat(34) = 6.12323399573677e-17*x263 - 1.0*x3*(x261*x35 + x262*x36) + x34*(x261*x37 + x262*x35);
    dyn_hbmat(35) = 6.12323399573677e-17*x274 - 1.0*x3*(x272*x35 + x273*x36) + x34*(x272*x37 + x273*x35);
    dyn_hbmat(36) = 0.2435*x0*(x300 + x302) - 0.2435*x3*(x298 + x299*x37) - 1.0*x3*(-0.4318*x190*x286 - 0.4318*x279 + x289*x36 - 2.64401243935914e-17*x290 + x296*x35 - 0.0934*x298 + x299*x87) + 6.12323399573677e-17*x303 + x34*(x289*x35 + x296*x37 + x297*x87 + 0.0934*x300);
    dyn_hbmat(37) = 0.2435*x0*(x325 + x328) - 0.2435*x3*(x324*x37 + x329) - 1.0*x3*(-0.4318*x190*x306 - 0.4318*x311 - 2.64401243935914e-17*x322 + x323*x35 + x324*x87 + x327*x36 - 0.0934*x329) + 6.12323399573677e-17*x331 + x34*(x323*x37 + 0.0934*x325 + x326*x87 + x327*x35);
    dyn_hbmat(38) = 0;
    dyn_hbmat(39) = 0;
    dyn_hbmat(40) = -1.0*x3*(x35*x356 + x357*x36) + x34*(x35*x357 + x356*x37) + 6.12323399573677e-17*x358;
    dyn_hbmat(41) = -1.0*x3*(x35*x379 + x36*x380) + x34*(x35*x380 + x37*x379) + 6.12323399573677e-17*x378;
    dyn_hbmat(42) = -1.0*x3*(x35*x398 + x36*x399) + x34*(x35*x399 + x37*x398) + 6.12323399573677e-17*x400;
    dyn_hbmat(43) = -1.0*x3*(x35*x417 + x36*x416) + x34*(x35*x416 + x37*x417) + 6.12323399573677e-17*x415;
    dyn_hbmat(44) = -1.0*x3*(x35*x429 + x36*x426) + x34*(x35*x426 + x37*x429) + 6.12323399573677e-17*x430;
    dyn_hbmat(45) = 0.2435*x0*(x442 + x446) - 0.2435*x3*(x37*x445 + x463) - 1.0*x3*(-0.4318*x190*x436 + x35*x462 + x36*x464 - 0.4318*x443 - 2.64401243935914e-17*x444 + x445*x87 - 0.0934*x463) + x34*(x35*x464 + x37*x462 + x441*x87 + 0.0934*x446) + 6.12323399573677e-17*x466;
    dyn_hbmat(46) = 0.2435*x0*(x480 + x483) - 0.2435*x3*(x37*x479 + x498) - 1.0*x3*(-0.4318*x190*x473 + x35*x496 + x36*x497 - 0.4318*x477 - 2.64401243935914e-17*x478 + x479*x87 - 0.0934*x498) + x34*(x35*x497 + x37*x496 + 0.0934*x480 + x482*x87) + 6.12323399573677e-17*x500;
    dyn_hbmat(47) = 0;
    dyn_hbmat(48) = 0;
    dyn_hbmat(49) = 0;
    dyn_hbmat(50) = 0;
    dyn_hbmat(51) = 0;
    dyn_hbmat(52) = x6;
    dyn_hbmat(53) = x17;
    dyn_hbmat(54) = x18;
    dyn_hbmat(55) = x25;
    dyn_hbmat(56) = x21;
    dyn_hbmat(57) = x31;
    dyn_hbmat(58) = x33;
    dyn_hbmat(59) = dq(2);
    dyn_hbmat(60) = sign(dq(2));
    dyn_hbmat(61) = x44;
    dyn_hbmat(62) = x55;
    dyn_hbmat(63) = x56;
    dyn_hbmat(64) = x64;
    dyn_hbmat(65) = x57;
    dyn_hbmat(66) = x78;
    dyn_hbmat(67) = x85;
    dyn_hbmat(68) = 0;
    dyn_hbmat(69) = 0;
    dyn_hbmat(70) = x106;
    dyn_hbmat(71) = x122;
    dyn_hbmat(72) = x131;
    dyn_hbmat(73) = x141;
    dyn_hbmat(74) = x150;
    dyn_hbmat(75) = x173;
    dyn_hbmat(76) = x192;
    dyn_hbmat(77) = 0;
    dyn_hbmat(78) = 0;
    dyn_hbmat(79) = x215;
    dyn_hbmat(80) = x232;
    dyn_hbmat(81) = x250;
    dyn_hbmat(82) = x263;
    dyn_hbmat(83) = x274;
    dyn_hbmat(84) = x303;
    dyn_hbmat(85) = x331;
    dyn_hbmat(86) = 0;
    dyn_hbmat(87) = 0;
    dyn_hbmat(88) = x358;
    dyn_hbmat(89) = x378;
    dyn_hbmat(90) = x400;
    dyn_hbmat(91) = x415;
    dyn_hbmat(92) = x430;
    dyn_hbmat(93) = x466;
    dyn_hbmat(94) = x500;
    dyn_hbmat(95) = 0;
    dyn_hbmat(96) = 0;
    dyn_hbmat(97) = 0;
    dyn_hbmat(98) = 0;
    dyn_hbmat(99) = 0;
    dyn_hbmat(100) = 0;
    dyn_hbmat(101) = 0;
    dyn_hbmat(102) = 0;
    dyn_hbmat(103) = 0;
    dyn_hbmat(104) = 0;
    dyn_hbmat(105) = 0;
    dyn_hbmat(106) = 0;
    dyn_hbmat(107) = 0;
    dyn_hbmat(108) = 0;
    dyn_hbmat(109) = x44;
    dyn_hbmat(110) = x55;
    dyn_hbmat(111) = x56;
    dyn_hbmat(112) = x64;
    dyn_hbmat(113) = x57;
    dyn_hbmat(114) = x77;
    dyn_hbmat(115) = x84;
    dyn_hbmat(116) = dq(3);
    dyn_hbmat(117) = sign(dq(3));
    dyn_hbmat(118) = x106;
    dyn_hbmat(119) = x122;
    dyn_hbmat(120) = x131;
    dyn_hbmat(121) = x141;
    dyn_hbmat(122) = x150;
    dyn_hbmat(123) = x171;
    dyn_hbmat(124) = x191;
    dyn_hbmat(125) = 0;
    dyn_hbmat(126) = 0;
    dyn_hbmat(127) = x215;
    dyn_hbmat(128) = x232;
    dyn_hbmat(129) = x250;
    dyn_hbmat(130) = x263;
    dyn_hbmat(131) = x274;
    dyn_hbmat(132) = x301;
    dyn_hbmat(133) = x330;
    dyn_hbmat(134) = 0;
    dyn_hbmat(135) = 0;
    dyn_hbmat(136) = x358;
    dyn_hbmat(137) = x378;
    dyn_hbmat(138) = x400;
    dyn_hbmat(139) = x415;
    dyn_hbmat(140) = x430;
    dyn_hbmat(141) = x465;
    dyn_hbmat(142) = x499;
    dyn_hbmat(143) = 0;
    dyn_hbmat(144) = 0;
    dyn_hbmat(145) = 0;
    dyn_hbmat(146) = 0;
    dyn_hbmat(147) = 0;
    dyn_hbmat(148) = 0;
    dyn_hbmat(149) = 0;
    dyn_hbmat(150) = 0;
    dyn_hbmat(151) = 0;
    dyn_hbmat(152) = 0;
    dyn_hbmat(153) = 0;
    dyn_hbmat(154) = 0;
    dyn_hbmat(155) = 0;
    dyn_hbmat(156) = 0;
    dyn_hbmat(157) = 0;
    dyn_hbmat(158) = 0;
    dyn_hbmat(159) = 0;
    dyn_hbmat(160) = 0;
    dyn_hbmat(161) = 0;
    dyn_hbmat(162) = 0;
    dyn_hbmat(163) = 0;
    dyn_hbmat(164) = 0;
    dyn_hbmat(165) = 0;
    dyn_hbmat(166) = x102;
    dyn_hbmat(167) = x113;
    dyn_hbmat(168) = x127;
    dyn_hbmat(169) = x140;
    dyn_hbmat(170) = x128;
    dyn_hbmat(171) = x166;
    dyn_hbmat(172) = x184;
    dyn_hbmat(173) = dq(4);
    dyn_hbmat(174) = sign(dq(4));
    dyn_hbmat(175) = x211;
    dyn_hbmat(176) = x229;
    dyn_hbmat(177) = x248;
    dyn_hbmat(178) = x260;
    dyn_hbmat(179) = x268;
    dyn_hbmat(180) = x294;
    dyn_hbmat(181) = x315;
    dyn_hbmat(182) = 0;
    dyn_hbmat(183) = 0;
    dyn_hbmat(184) = x351;
    dyn_hbmat(185) = x377;
    dyn_hbmat(186) = x397;
    dyn_hbmat(187) = x412;
    dyn_hbmat(188) = x428;
    dyn_hbmat(189) = x459;
    dyn_hbmat(190) = x491;
    dyn_hbmat(191) = 0;
    dyn_hbmat(192) = 0;
    dyn_hbmat(193) = 0;
    dyn_hbmat(194) = 0;
    dyn_hbmat(195) = 0;
    dyn_hbmat(196) = 0;
    dyn_hbmat(197) = 0;
    dyn_hbmat(198) = 0;
    dyn_hbmat(199) = 0;
    dyn_hbmat(200) = 0;
    dyn_hbmat(201) = 0;
    dyn_hbmat(202) = 0;
    dyn_hbmat(203) = 0;
    dyn_hbmat(204) = 0;
    dyn_hbmat(205) = 0;
    dyn_hbmat(206) = 0;
    dyn_hbmat(207) = 0;
    dyn_hbmat(208) = 0;
    dyn_hbmat(209) = 0;
    dyn_hbmat(210) = 0;
    dyn_hbmat(211) = 0;
    dyn_hbmat(212) = 0;
    dyn_hbmat(213) = 0;
    dyn_hbmat(214) = 0;
    dyn_hbmat(215) = 0;
    dyn_hbmat(216) = 0;
    dyn_hbmat(217) = 0;
    dyn_hbmat(218) = 0;
    dyn_hbmat(219) = 0;
    dyn_hbmat(220) = 0;
    dyn_hbmat(221) = 0;
    dyn_hbmat(222) = 0;
    dyn_hbmat(223) = x200;
    dyn_hbmat(224) = x227;
    dyn_hbmat(225) = x239;
    dyn_hbmat(226) = x257;
    dyn_hbmat(227) = x240;
    dyn_hbmat(228) = x284;
    dyn_hbmat(229) = x313;
    dyn_hbmat(230) = dq(5);
    dyn_hbmat(231) = sign(dq(5));
    dyn_hbmat(232) = x348;
    dyn_hbmat(233) = x374;
    dyn_hbmat(234) = x389;
    dyn_hbmat(235) = x410;
    dyn_hbmat(236) = x424;
    dyn_hbmat(237) = x455;
    dyn_hbmat(238) = x488;
    dyn_hbmat(239) = 0;
    dyn_hbmat(240) = 0;
    dyn_hbmat(241) = 0;
    dyn_hbmat(242) = 0;
    dyn_hbmat(243) = 0;
    dyn_hbmat(244) = 0;
    dyn_hbmat(245) = 0;
    dyn_hbmat(246) = 0;
    dyn_hbmat(247) = 0;
    dyn_hbmat(248) = 0;
    dyn_hbmat(249) = 0;
    dyn_hbmat(250) = 0;
    dyn_hbmat(251) = 0;
    dyn_hbmat(252) = 0;
    dyn_hbmat(253) = 0;
    dyn_hbmat(254) = 0;
    dyn_hbmat(255) = 0;
    dyn_hbmat(256) = 0;
    dyn_hbmat(257) = 0;
    dyn_hbmat(258) = 0;
    dyn_hbmat(259) = 0;
    dyn_hbmat(260) = 0;
    dyn_hbmat(261) = 0;
    dyn_hbmat(262) = 0;
    dyn_hbmat(263) = 0;
    dyn_hbmat(264) = 0;
    dyn_hbmat(265) = 0;
    dyn_hbmat(266) = 0;
    dyn_hbmat(267) = 0;
    dyn_hbmat(268) = 0;
    dyn_hbmat(269) = 0;
    dyn_hbmat(270) = 0;
    dyn_hbmat(271) = 0;
    dyn_hbmat(272) = 0;
    dyn_hbmat(273) = 0;
    dyn_hbmat(274) = 0;
    dyn_hbmat(275) = 0;
    dyn_hbmat(276) = 0;
    dyn_hbmat(277) = 0;
    dyn_hbmat(278) = 0;
    dyn_hbmat(279) = 0;
    dyn_hbmat(280) = x345;
    dyn_hbmat(281) = x366;
    dyn_hbmat(282) = x388;
    dyn_hbmat(283) = x405;
    dyn_hbmat(284) = x385;
    dyn_hbmat(285) = x449;
    dyn_hbmat(286) = x486;
    dyn_hbmat(287) = dq(6);
    dyn_hbmat(288) = sign(dq(6));

%;
    dyn_hbmat = transpose(dyn_hbmat);

end