%% intermediate function that assign different parameter to different controls
function finalerr = mymove(traj, mode, para, ur5)
    finalerr = 0;

    for i = 1:size(traj,3)

        if mode == 1
            finalerr = invKinControl(traj(:,:,i), para(1), ur5);
            if(finalerr == -1)
                return
            end

        elseif mode == 2
            finalerr = invJacControl(traj(:,:,i),para(2), ur5);
            if(finalerr == -1)
                return
            end

        elseif mode == 3
            finalerr = transJacControl(traj(:,:,i),para(3),ur5);
            if(finalerr == -1)
                return
            end
        end

    end

end