clear all
videoPathList{1} = 'GOPR0056.avi'; 
videoPathList{2} = 'GOPR0083.avi'; 
for i = 1:numel(videoPathList)
    videoObj{i} = vision.VideoFileReader(videoPathList{i}, 'AudioOutputPort', true);
%     audioObj{i} = dsp.AudioFileReader(videoPathList{i}); 
    audio{i} = []; 
    while ~videoObj{i}.isDone()
        [~, A] = videoObj{i}.step(); 
        A = max(A); 
        audio{i} = [audio{i}; A]; 
    end
    audio{i} = mean(audio{i}, 2); 
end