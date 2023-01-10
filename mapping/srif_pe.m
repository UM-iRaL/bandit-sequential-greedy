function S = srif_pe(S,A,chol_inv_W,H,chol_inv_V)
% S = srif_pe(S,A,chol_inv_W,H,chol_inv_V)
%	
%	S = cholesky factor of the info matrix, i.e. Omega = S'*S
%
%   Computes the update step of the square root information filter
%


if( size(S,1) ~= size(S,2) )
    error('[srif_pe]: Works for square cholesky factors only!');
end

dx = size(A,1);         % number of mobile variables
ds = size(S,2) - dx;    % number of static variables
dz = size(H,1);         % number of observations


X_S = S(1:dx,1:dx);
Y_S = S(1:dx,(dx+1):end);
if( issparse(S) )
    if( issparse(A) )
        T1 = X_S/A;
    else
        T1 = X_S/sparse(A);
    end
    if( issparse(H) )
        T2 = chol_inv_V*H;
    else
        T2 = sparse(chol_inv_V*H);
    end
    B = [sparse(chol_inv_W),sparse(dx,dx+ds);
        T1,T1,Y_S;
        sparse(ds,2*dx), S((dx+1):end,(dx+1):end);
        sparse(dz,dx), T2];
    B = qr(B);
else
    T = X_S/A;
    B = [chol_inv_W,zeros(dx,dx+ds);
        T,T,Y_S;
        zeros(ds,2*dx), S((dx+1):end,(dx+1):end);
        zeros(dz,dx), chol_inv_V*H];
    B = triu(qr(B));
end
S = B((dx+1):(dx+ds+dx),(dx+1):(dx+ds+dx));
    
% if(ds > 0)
% else
%     if( issparse(S) )
% 
%         if( issparse(A) )
%             T1 = S/A;
%         else
%             T1 = S/sparse(A);
%         end
%         
%         if( issparse(H) )
%             T2 = chol_inv_V*H;
%         else
%             T2 = sparse(chol_inv_V*H);
%         end
%         B = [sparse(chol_inv_W),sparse(dx,dx);
%             T1,T1; sparse(dz,dx), T2];
%         B = qr(B);
%     else
%         T = S/A;
%         B = [chol_inv_W, zeros(dx,dx);
%             T,T; zeros(dz,dx), chol_inv_V*H];
%         B = triu(qr(B));
%     end
%     S = B((dx+1):(2*dx),(dx+1):(2*dx));
% end

end
