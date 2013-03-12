clear all

marginTime = 5; 



videoPathList{1} = '../image/6cameras/09748.avi'; 
videoPathList{2} = '../image/6cameras/09749.avi'; 
videoPathList{3} = '../image/6cameras/09753.avi'; 
videoPathList{4} = '../image/6cameras/09754.avi'; 
videoPathList{5} = '../image/6cameras/09756.avi'; 
videoPathList{6} = '../image/6cameras/nolabel.avi'; 


for i = 1:numel(videoPathList)
%     videoObj{i} = vision.VideoFileReader(videoPathList{i}, 'AudioOutputPort', true);
    audioObj{i} = dsp.AudioFileReader(videoPathList{i}); 
    marginFrames = round(marginTime * audioObj{i}.SampleRate / audioObj{i}.SamplesPerFrame); 
    
    audio{i} = []; 
    while ~audioObj{i}.isDone()
        A = audioObj{i}.step(); 
        A = max(A); 
        audio{i} = [audio{i}; A]; 
    end
    audio{i} = mean(audio{i}, 2); 
    audio{i}(1:marginFrames) = 0; 
    audio{i}(end - marginFrames + 1 :end) = 0; 
end



startTime = 0; 
endTime = length(audio{1}) * audioObj{1}.SamplesPerFrame / audioObj{1}.SampleRate; 
for i = 2:numel(audioObj)
    assert(audioObj{i}.SampleRate == audioObj{1}.SampleRate)
    cor = imfilter(audio{1}, audio{i}, 'full', 'corr'); 
    [~, imax] = max(cor); 
    startTime(i) = (imax - length(audio{i})) * audioObj{i}.SamplesPerFrame / audioObj{i}.SampleRate; 
    endTime(i) = length(audio{i}) * audioObj{i}.SamplesPerFrame / audioObj{i}.SampleRate; 
end 

latestStart = max(startTime); 
waitTime = latestStart - startTime; 
durTime = endTime - startTime; 
minDur = floor(min(durTime)); 

frameSkip = 30; 

color = 'rgbmyc'

for i = 1:numel(videoPathList)
    videoObj{i} = vision.VideoFileReader(videoPathList{i}, 'AudioOutputPort', true);
    waitFrames = waitTime(i) * videoObj{i}.info.VideoFrameRate; 
    endFrames = endTime(i) * videoObj{i}.info.VideoFrameRate; 
    durFrames = minDur * videoObj{i}.info.VideoFrameRate; 
    maxStamp = floor(durFrames / frameSkip); 
    
    audio{i} = []; 
    count = 0; 
    timeStamp = 1; 
    while ~videoObj{i}.isDone() && timeStamp < maxStamp % count < durFrames + waitFrames
        count = count + 1;
        [I, A] = videoObj{i}.step(); 
        if count < waitFrames
            continue; 
        end
        A = max(A);
        audio{i} = [audio{i}; A]; 

        
        if mod(count, frameSkip) ~= 0
            continue; 
        end
        
        display([num2str(i), '-', num2str(timeStamp)]); 
        imwrite(I, ['../image/6cameras/images/', num2str(timeStamp), '-', num2str(i), '.png']); 
        timeStamp = timeStamp + 1;
    end
    audio{i} = mean(audio{i}, 2); 
    
    plot(audio{i}, color(i)), hold on; 
    
end

