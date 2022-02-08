function [observed, table] = associate_tag(j, table, idx)
% if all entries corresponding to the jth tag are zero, we have not
% observed it before
%     B = P(idx,idx);
%     
%     if B == zeros(2)
%         %if all enries are zero its a new tag
%         observed = false;
%     else
%         %if entries are non-zero its a known tag
%         observed = true;
%     end
    
    if table(j) == 0
        observed = false;
        table(j) = 1;
    else
        observed = true;
    end
    
end