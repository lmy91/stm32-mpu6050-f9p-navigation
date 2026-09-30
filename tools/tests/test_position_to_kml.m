function test_position_to_kml
%TEST_POSITION_TO_KML Base MATLAB regression checks, no Mapping Toolbox.
% Run: addpath('tools/tests'); test_position_to_kml
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(root,'tools'));
test_dir=tempname;
mkdir(test_dir);
cleanup=onCleanup(@()remove_test_outputs(test_dir)); %#ok<NASGU>
t=[10.2;10.8;11.4;12;12.6;13.2];
llh=[30+0.001*t,120+0.002*t,40+0.1*t];
label='组合 & <轨迹> "A" ''B''';
file=fullfile(test_dir,'test_utf8.kml');
[pos,time]=position_to_kml(llh,t,file,'AngleUnit','deg','Name',label);
assert(isequal(time,[11;12;13]),'Output must be an integer-second grid without extrapolation.');
expected=[30+0.001*time,120+0.002*time,40+0.1*time];
assert(max(abs(pos-expected),[],'all')<1e-10,'Linear resampling is incorrect.');
doc=xmlread(file);
assert(strcmp(char(doc.getDocumentElement.getAttribute('xmlns')),'http://www.opengis.net/kml/2.2'));
assert(strcmp(char(doc.getElementsByTagName('name').item(0).getTextContent),label), ...
    'UTF-8 text or XML escaping is incorrect.');
assert(strcmp(char(doc.getElementsByTagName('altitudeMode').item(0).getTextContent),'clampToGround'));
line=doc.getElementsByTagName('LineString').item(0);
coords=char(line.getElementsByTagName('coordinates').item(0).getTextContent);
numbers=sscanf(strrep(coords,',',' '),'%f',[3 Inf])';
assert(isequal(size(numbers),[3 3]) && max(abs(numbers-pos(:,[2 1 3])),[],'all')<1e-9, ...
    'KML must contain longitude,latitude,height in that order.');

rad=llh;rad(:,1:2)=rad(:,1:2)*pi/180;
[pos_rad,time_rad]=position_to_kml(rad,t,fullfile(test_dir,'rad.kml'),'AltitudeMode','absolute');
assert(isequal(time_rad,time) && max(abs(pos_rad-pos),[],'all')<1e-10);
doc_rad=xmlread(fullfile(test_dir,'rad.kml'));
assert(strcmp(char(doc_rad.getElementsByTagName('altitudeMode').item(0).getTextContent),'absolute'));

[meridian,~]=position_to_kml([0 179.8 10;0 -179.8 10],[0;2], ...
    fullfile(test_dir,'meridian.kml'),'AngleUnit','deg');
assert(abs(abs(meridian(2,2))-180)<1e-10,'Longitude interpolation crossed Greenwich incorrectly.');

[one,one_t]=position_to_kml([30 120 10;30 120 11],[.2;1.2], ...
    fullfile(test_dir,'single_epoch.kml'),'AngleUnit','deg');
doc_one=xmlread(fullfile(test_dir,'single_epoch.kml'));
assert(size(one,1)==1 && one_t==1 && doc_one.getElementsByTagName('LineString').getLength==0, ...
    'A single exported epoch must use a Point rather than an invalid one-vertex LineString.');
assert(doc_one.getElementsByTagName('Point').getLength>=1);

expect_error(@()position_to_kml(llh,[10.2;10.2;11.4;12;12.6;13.2], ...
    fullfile(test_dir,'bad_time.kml'),'AngleUnit','deg'),'position_to_kml:InvalidTime');
expect_error(@()position_to_kml([91 120 10;91 120 11],[0;1], ...
    fullfile(test_dir,'bad_position.kml'),'AngleUnit','deg'),'position_to_kml:InvalidCoordinates');
expect_error(@()position_to_kml([30 120 10;30 120 11],[.1;.2], ...
    fullfile(test_dir,'no_epoch.kml'),'AngleUnit','deg'),'position_to_kml:NoSampleEpoch');
expect_error(@()position_to_kml(llh,t,fullfile(test_dir,'bad_color.kml'), ...
    'AngleUnit','deg','LineColor','red'),'position_to_kml:InvalidColor');
fprintf('position_to_kml checks passed: 1 Hz, no extrapolation, LLH order, radians/degrees, UTF-8/XML, altitude modes, meridian, single epoch, invalid inputs.\n');
end

function expect_error(f,id)
try
    f();
catch err
    assert(strcmp(err.identifier,id),'Unexpected error identifier: %s',err.identifier);
    return;
end
error('Expected validation error %s was not raised.',id);
end

function remove_test_outputs(test_dir)
% Only remove the fresh test directory created above, never the temp root.
root=char(java.io.File(tempdir).getCanonicalPath);
target=char(java.io.File(test_dir).getCanonicalPath);
assert(startsWith(lower(target),lower([root filesep])) && ~strcmpi(target,root), ...
    'Refusing to remove a test output directory outside the temporary root.');
if isfolder(target);rmdir(target,'s');end
end
