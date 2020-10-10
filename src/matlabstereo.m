RD1 = stereoParams.CameraParameters1.RadialDistortion;
TD1 = stereoParams.CameraParameters1.TangentialDistortion;
D1 = [RD1(1), RD1(2), TD1(1), TD1(2), RD1(3)];
% D1 = [RD1(1), RD1(2), TD1(1), TD1(2)];
K1 = stereoParams.CameraParameters1.IntrinsicMatrix';

RD2 = stereoParams.CameraParameters2.RadialDistortion;
TD2 = stereoParams.CameraParameters2.TangentialDistortion;
D2 = [RD2(1), RD2(2), TD2(1), TD2(2), RD2(3)];
% D2 = [RD2(1), RD2(2), TD2(1), TD2(2)];
K2 = stereoParams.CameraParameters2.IntrinsicMatrix';

size = stereoParams.CameraParameters1.ImageSize;

rot2 = stereoParams.RotationOfCamera2;
trans2 = stereoParams.TranslationOfCamera2;
T2=eye(4);
T2(1:3,1:3)=rot2;
T2(1:3,4)=trans2;

T1=inv(T2);
rot1=T1(1:3,1:3);
trans1=T1(1:3,4);

X = struct('K1',K1 ,'D1', D1,'K2',K2 ,'D2', D2,'size',size, 'rot1',rot1,'trans1',trans1, 'rot2',rot2,'trans2',trans2,'T1',T1,'T2',T2);

file = './cali_matlab.yaml';
YAML.write(file, X ); % save X to a yaml file
X = YAML.read(file); % load X from a yaml file
disp(X)