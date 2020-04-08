function xsensTOValues = xsensTO(toeZPos, checkRange)
xsensHCValues = [];
%The check range number might have to be adjusted, look at the XSENS Toe
%and Heel position graph to see if there are obvious points missing or if
%there are too many points. Increase the number to get rid of extra points,
%decrease to accept more points.

% ID toe-off using toe position data
xsensTOValues = [];

[a,b] = size(toeZPos(:,:));
for len = checkRange + 1:a-checkRange
    counter = 0;
    for checkThese = 1:checkRange
        if toeZPos(len,1) < toeZPos(len+checkThese,1) && toeZPos(len,1) < toeZPos(len-checkThese,1)
            counter = counter + 1;
        end
    end
    if counter == checkRange;
        [a,b] = size(xsensTOValues);
        xsensTOValues(a+1,1) = len;
    end
end

figure
hold on
plot(toeZPos(:,1), 'r-')
plot(xsensTOValues, toeZPos(xsensTOValues,1), 'ro', 'LineWidth', 2)
legend('Toe Z Pos.', 'TO')
xlabel('XSENS Frame/Time')
ylabel('Height (m)')
title('XSENS Toe and Heel Data with Identified HC and TO Points')
end