function [pos_out, time_out] = position_to_kml(pos, time_s, kml_file, varargin)
%POSITION_TO_KML Export WGS84 positions as a uniformly sampled KML path.
%   [pos_out,time_out] = position_to_kml(pos,time_s,kml_file)
%   pos: N-by-3 [latitude,longitude,height], radians/radians/metres by default.
%   time_s: N increasing sample epochs in seconds (e.g. GPS TOW), N >= 2.
%   pos_out: exported [latitude_deg,longitude_deg,height_m].
%   time_out: uniform epochs, default 1 Hz, aligned to integer seconds.
%
% Name/value options:
%   AngleUnit       'rad' (PSINS default) or 'deg'
%   SamplePeriod_s  1 (no extrapolation; linear interpolation within input)
%   AltitudeMode    'clampToGround' (default), 'absolute', 'relativeToGround'
%   Name            'Navigation trajectory'
%   LineColor       'ffff5500' (KML aabbggrr hex, opaque blue)
%   LineWidth       3
%
% Example for an existing PSINS antenna AVP log:
%   position_to_kml(avpL(:,7:9),avpL(:,10),'combined_navigation_1hz.kml');
% Prefer continuous propagation logs to feedback-only logs when available.
% KML coordinates are longitude,latitude,height; input order stays PSINS LLH.
% clampToGround keeps stored heights but ignores them in terrain display.
% absolute expects sea-level heights, NOT unconverted ellipsoid heights.
% No Mapping Toolbox, external icons or online coordinate conversion needed.

validateattributes(pos,{'numeric'},{'real','finite','2d','nonempty'},mfilename,'pos');
validateattributes(time_s,{'numeric'},{'real','finite','vector','nonempty'},mfilename,'time_s');
assert(size(pos,2)==3 && size(pos,1)>=2,'position_to_kml:InvalidPosition', ...
    'pos must have at least two rows of [latitude,longitude,height].');
time_s=double(time_s(:));pos=double(pos);
assert(numel(time_s)==size(pos,1) && all(diff(time_s)>0), ...
    'position_to_kml:InvalidTime','Times must match positions and increase strictly.');
assert(text_scalar(kml_file) && ~isempty(char(kml_file)), ...
    'position_to_kml:InvalidFilename','A nonempty scalar output filename is required.');

p=inputParser;
p.FunctionName=mfilename;
addParameter(p,'AngleUnit','rad',@text_scalar);
addParameter(p,'SamplePeriod_s',1,@(x)isnumeric(x)&&isscalar(x)&&isreal(x)&&isfinite(x)&&x>0);
addParameter(p,'AltitudeMode','clampToGround',@text_scalar);
addParameter(p,'Name','Navigation trajectory',@text_scalar);
addParameter(p,'LineColor','ffff5500',@text_scalar);
addParameter(p,'LineWidth',3,@(x)isnumeric(x)&&isscalar(x)&&isreal(x)&&isfinite(x)&&x>0);
parse(p,varargin{:});opt=p.Results;
unit=validatestring(char(opt.AngleUnit),{'rad','deg'},mfilename,'AngleUnit');
mode=validatestring(char(opt.AltitudeMode), ...
    {'clampToGround','absolute','relativeToGround'},mfilename,'AltitudeMode');
color=char(opt.LineColor);
assert(~isempty(regexp(color,'^[0-9A-Fa-f]{8}$','once')), ...
    'position_to_kml:InvalidColor','LineColor must be eight KML aabbggrr hex digits.');
if strcmp(unit,'rad');pos(:,1:2)=pos(:,1:2)*180/pi;end
assert(all(abs(pos(:,1))<=90) && all(abs(pos(:,2))<=180), ...
    'position_to_kml:InvalidCoordinates','WGS84 latitude/longitude are out of range; check AngleUnit.');

step=double(opt.SamplePeriod_s);
first=ceil(time_s(1)/step);last=floor(time_s(end)/step);
assert(last>=first,'position_to_kml:NoSampleEpoch', ...
    'The input interval contains no complete sampling-grid epoch.');
time_out=(first:last)'*step;
% Unwrap longitude before interpolation, avoiding an artificial path through
% Greenwich when input crosses the +/-180 degree meridian.
pos(:,2)=unwrap(pos(:,2)*pi/180)*180/pi;
pos_out=interp1(time_s,pos,time_out,'linear');
assert(all(isfinite(pos_out),'all'),'position_to_kml:ResamplingFailed', ...
    'Resampling produced invalid coordinates; no extrapolation is allowed.');
pos_out(:,2)=mod(pos_out(:,2)+180,360)-180;

name=xml_text(opt.Name);
kml_file=char(kml_file);
[folder,~,extension]=fileparts(kml_file);
assert(strcmpi(extension,'.kml'),'position_to_kml:InvalidExtension', ...
    'The output filename must have a .kml extension.');
if ~isempty(folder) && ~isfolder(folder)
    [ok,msg]=mkdir(folder);
    assert(ok,'position_to_kml:CreateDirectoryFailed','Cannot create output directory: %s',msg);
end
[fid,msg]=fopen(kml_file,'w','n','UTF-8');
assert(fid>=0,'position_to_kml:OpenFailed','Cannot open KML output: %s',msg);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'<?xml version="1.0" encoding="UTF-8"?>\n');
fprintf(fid,'<kml xmlns="http://www.opengis.net/kml/2.2">\n<Document>\n');
fprintf(fid,'  <name>%s</name>\n',name);
fprintf(fid,'  <description>Uniformly sampled WGS84 navigation positions. Heights in metres; %s display.</description>\n',mode);
fprintf(fid,'  <Style id="track"><LineStyle><color>%s</color><width>%.6g</width></LineStyle></Style>\n',lower(color),opt.LineWidth);
fprintf(fid,'  <Style id="start"><IconStyle><color>ff00ff00</color></IconStyle></Style>\n');
fprintf(fid,'  <Style id="end"><IconStyle><color>ff0000ff</color></IconStyle></Style>\n');
fprintf(fid,'  <ExtendedData>\n');
fprintf(fid,'    <Data name="sample_period_s"><value>%.12g</value></Data>\n',step);
fprintf(fid,'    <Data name="point_count"><value>%d</value></Data>\n',numel(time_out));
fprintf(fid,'    <Data name="time_start_s"><value>%.9f</value></Data>\n',time_out(1));
fprintf(fid,'    <Data name="time_end_s"><value>%.9f</value></Data>\n',time_out(end));
fprintf(fid,'  </ExtendedData>\n');
fprintf(fid,'  <Placemark><name>%s</name><styleUrl>#track</styleUrl>\n',name);
if numel(time_out)>=2
    fprintf(fid,'    <LineString><tessellate>%d</tessellate><altitudeMode>%s</altitudeMode>\n',strcmp(mode,'clampToGround'),mode);
    fprintf(fid,'      <coordinates>\n');
    fprintf(fid,'        %.10f,%.10f,%.4f\n',pos_out(:,[2 1 3])');
    fprintf(fid,'      </coordinates>\n    </LineString>\n');
else
    fprintf(fid,'    <Point><altitudeMode>%s</altitudeMode><coordinates>%.10f,%.10f,%.4f</coordinates></Point>\n', ...
        mode,pos_out(1,2),pos_out(1,1),pos_out(1,3));
end
fprintf(fid,'  </Placemark>\n');
endpoint(fid,'Start','start',pos_out(1,:),mode,time_out(1));
if numel(time_out)>1;endpoint(fid,'End','end',pos_out(end,:),mode,time_out(end));end
fprintf(fid,'</Document>\n</kml>\n');
[msg,err]=ferror(fid);
assert(err==0,'position_to_kml:WriteFailed','KML write failed: %s',msg);
end

function endpoint(fid,name,style,pos,mode,t)
fprintf(fid,'  <Placemark><name>%s</name><styleUrl>#%s</styleUrl>',name,style);
fprintf(fid,'<description>Sample epoch: %.9f s</description>',t);
fprintf(fid,'<Point><altitudeMode>%s</altitudeMode><coordinates>%.10f,%.10f,%.4f</coordinates></Point></Placemark>\n', ...
    mode,pos(2),pos(1),pos(3));
end

function tf=text_scalar(value)
tf=(ischar(value)&&(isrow(value)||isempty(value))) || ...
    (isstring(value)&&isscalar(value)&&~ismissing(value));
end

function out=xml_text(value)
out=char(value);
assert(~any(double(out)<32 & ~ismember(double(out),[9 10 13])), ...
    'position_to_kml:InvalidText','KML text contains invalid XML control characters.');
out=strrep(out,'&','&amp;');out=strrep(out,'<','&lt;');
out=strrep(out,'>','&gt;');out=strrep(out,'"','&quot;');
out=strrep(out,'''','&apos;');
end
