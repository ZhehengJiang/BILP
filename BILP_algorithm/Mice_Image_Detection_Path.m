%% Path to load the image frames and detections

% load labels
load('.\Data\MICE\labels.mat');
% Address path to the image frames 
Detection_address=fullfile('.','Data','MICE','detections');

% Image info
testData=testData(1:end,:);
num_file = size(testData,1);
for i =1: size(testData,1)
    jpda_data= strfind(testData.imageFilename{i},'mouse_images');
    testData.imageFilename{i} = fullfile('.\Data\MICE\mouse_images',testData.imageFilename{i}(jpda_data(2):end));
end
info = imfinfo(testData.imageFilename{1});
u_image=info(1).Height;
v_image=info(1).Width;
cl_image=class(imread(testData.imageFilename{1}));