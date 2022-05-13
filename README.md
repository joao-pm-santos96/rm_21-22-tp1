# RM-TP1
Localização Probabilística de Robôs com Redundância de Faróis

## BeaconDetection

```
function [B]=BeaconDetection(N,P,obsNoise)
%
% Function that does two main tasks:
% Creates a set of up to N beacons in a fixed arrangement.
% Returns informations of those beacons as described.
%
% INPUTS:
% N - number of beacons to create/use (N>3) but large values may not be respected
% P - current estimated position (x,y,a). (0,0,0) if absent.
% obsNoise - observation noise [range, heading]. If not passed, use a default value
%
% OUTPUT
% B - array of structures with data from beacons:
% B.X - real beacon X position (fixed and known)
% B.Y - real beacon Y position (fixed and known)
% B.d - measured distance (with uncertainty)
% B.a - measured angle (with uncertainty)
% B.dn - sigma in B.d (either the passed in obsNoise or a default)
% B.an - sigma in B.a (either the passed in obsNoise or a default)
%
% NOTES:
% Occasionally B.d and B.a are not available (too close or to far from beacon)
% and in those cases their value is NaN and sensorial data is not available.
```

## Notable Functions

- pchip