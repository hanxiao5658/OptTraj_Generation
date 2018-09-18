function  LG_ani( Q,P )
%LG_ANI Summary of this function goes here
%   Detailed explanation goes here
phi = 0*Q.x;
theta = 0*phi;
psi = 0*phi;
posQ = [Q.x,Q.y,Q.z];
posP = [P.x,P.y,P.z];
R = euler2rotMat(phi,theta,psi);

SamplePlotFreq = 1;
isCreateAVI = false;
isFixView = false;
LG_6DoFAnimation(posQ,R,posP,SamplePlotFreq,'DotsOnly',isCreateAVI,isFixView);

end

