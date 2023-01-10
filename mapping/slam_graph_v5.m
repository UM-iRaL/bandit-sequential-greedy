classdef slam_graph_v5 < handle
    properties (SetAccess = protected, GetAccess = public)
        means = {};
        inv_sqrt_covs = {};
        da = {};
        ii = {};
        jj = {};
        type = {};
        num_msr = 0;
        msr_dim = {};
        %num_var = 0;
        %var_dim = 0;
        
        % map converting from measurement type to function handles for
        % measurements and jacobians
        type_to_handle;
        vardim_all = []; % dimensions of all variables
        vardim_lin = []; % dimensions of linearized vars
        all_to_lin_idx = [];  % map from variable index to linearized index 
        A = sparse(0,0);
        b = sparse(0,0);
    end
    
    methods
        function this = slam_graph_v5()
            this.type_to_handle = containers.Map('KeyType','double','ValueType','any');
        end
        
        function add_unary(this, tp, err, id, x_dim, J, mean, inv_sqrt_cov)
            % h = handle to measurement function
            % H = handle to jacobian of measurement function
            this.means{end+1} = mean;
            this.inv_sqrt_covs{end+1} = inv_sqrt_cov;
            this.da{end+1} = id;
            
            z_dim = length(mean);
            [idx1,idx2] = ndgrid(1:z_dim,1:x_dim);
            this.ii{end+1} = {idx1(:)};
            this.jj{end+1} = {idx2(:)};
            this.type{end+1} = tp;
            
            if( ~isKey(this.type_to_handle, tp) )
                this.type_to_handle(tp) = {err;J};
            end
            
            this.num_msr = this.num_msr + 1;
            this.msr_dim{end+1} = z_dim;
        end
        
        function add_binary(this, tp, err, id1, dim1, J1, id2, dim2, J2, mean, inv_sqrt_cov)
            this.means{end+1} = mean;
            this.inv_sqrt_covs{end+1} = inv_sqrt_cov;
            this.da{end+1} = [id1;id2];
            
            z_dim = length(mean);
            [mat1,mat2] = ndgrid(1:z_dim,1:dim1);
            [mat3,mat4] = ndgrid(1:z_dim,1:dim2);
            
            this.ii{end+1} = {mat1(:);mat3(:)};
            this.jj{end+1} = {mat2(:);mat4(:)};
            this.type{end+1} = tp;
            
            if( ~isKey(this.type_to_handle, tp) )
                this.type_to_handle(tp) = {err;J1;J2};
            end
            
            this.num_msr = this.num_msr + 1;
            this.msr_dim{end+1} = z_dim;
        end
        
        function linearize(this,estm,valid_idx)
            % valid_idx = logical array indicating over which variables to linearize
            mdims = cumsum([0;cat(1,this.msr_dim{:})]);
            
            num_var = length(estm);
            this.vardim_all = zeros(num_var,1);
            for kk = 1:num_var
                this.vardim_all(kk) = length(estm{kk});
            end
            cvdims = cumsum([0;this.vardim_all]);
            
            this.b = cell(this.num_msr,1);
            
            all_tp = cat(1,this.type{:});
            types = keys(this.type_to_handle);
            fhands = values(this.type_to_handle);
            
            num_types = length(types);
            
            ijs_A = cell(num_types,3);
            for tp = 1:num_types
                valid = (all_tp == types{tp});
                m = cat(2,this.means{valid});
                cov = cat(3,this.inv_sqrt_covs{valid});
                tmp = fhands{tp};
                err = tmp{1};
                
                dav = cat(2,this.da{valid});    % binary/unary x num_valid
                ii_tmp = cat(2,this.ii{valid}); % binary/unary x num_valid
                jj_tmp = cat(2,this.jj{valid}); % binary/unary x num_valid
                
                if( size(dav,1) == 1)
                    % unary
                    x = cat(2,estm{dav(1,:)});  % num_dim x num_x
                    res = -err(m,x);
                    J1 = tmp{2};
                    
                    ijs_A{tp,1} = reshape( bsxfun(@plus,cat(2,ii_tmp{:}), transpose(mdims(valid))), [], 1);
                    ijs_A{tp,2} = reshape( bsxfun(@plus,cat(2,jj_tmp{:}), transpose(cvdims(dav(1,:)))), [], 1);
                    ijs_A{tp,3} = reshape( mtimesx(cov,J1(x)), [], 1);
                    
                else
                    % binary
                    x = cat(2,estm{dav(1,:)});
                    y = cat(2,estm{dav(2,:)});
                    res = -err(m,x,y);
                    J1 = tmp{2};
                    J2 = tmp{3};
                    
                    ijs_A{tp,1} = [reshape( bsxfun(@plus,cat(2,ii_tmp{1,:}), transpose(mdims(valid))), [], 1);...
                        reshape( bsxfun(@plus,cat(2,ii_tmp{2,:}), transpose(mdims(valid))), [], 1)];
                    
                    ijs_A{tp,2} = [reshape( bsxfun(@plus,cat(2,jj_tmp{1,:}), transpose(cvdims(dav(1,:)))), [], 1);
                        reshape( bsxfun(@plus,cat(2,jj_tmp{2,:}), transpose(cvdims(dav(2,:)))), [], 1)];
                    
                    ijs_A{tp,3} = [ reshape( mtimesx(cov,J1(x,y)), [], 1);...
                        reshape( mtimesx(cov,J2(x,y)), [], 1)];
                    % it is possible that we do not need to compute both
                    % jacobians if we are interested in a subset of
                    % variables
                end
                
                [r_dim,r_num] = size(res);
                b_val = mtimesx(cov,reshape(res,r_dim,1,[]));
                
                b_cell = cell(r_num,1);
                for kk = 1:r_num
                    b_cell{kk} = b_val(:,kk);
                end
                
                this.b(valid) = b_cell;
            end
            
            this.b = cat(1,this.b{:}); % covert to vector
            
            this.A = sparse( cat(1,ijs_A{:,1}), cat(1,ijs_A{:,2}), cat(1,ijs_A{:,3}),...
                mdims(end),cvdims(end) );
            
            % remove unwanted columns
            % TODO: this can be improved by removing them before the matrix
            % creation
            if( nargin > 2 )
                %assert(length(vdims) == length(valid_idx)+1);
                this.A = this.A(:,rude(this.vardim_all,valid_idx));
                this.vardim_lin = this.vardim_all(valid_idx);
                this.all_to_lin_idx = zeros(num_var,1);
                this.all_to_lin_idx(valid_idx) = 1:sum(valid_idx);
            else
                this.vardim_lin = this.vardim_all;
                this.all_to_lin_idx = 1:num_var;
            end
        end
        
        function estm = solve(this, estm, lambda, max_iter, valid_idx)
            % valid_idx = logical array indicating over which variables to optimize
            if( nargin < 4 )
                max_iter = 100;
            end
            
            if( nargin < 3 )
                lambda = 0.01;
            end
            
            lambda_max=1e7;
            solver_tol = 1e-4;
            if( nargin > 4 )
                this.linearize(estm,valid_idx);
            else
                this.linearize(estm);
            end
            current_error = norm(this.b);
            
            step_backs=0;
            i=0;
            done=0;
            while ~done
                i=i+1;
                %fprintf('current_error = %f\n',current_error);
                
                dv = this.solve_iter(this.A,this.b,lambda);
                
                % update estimate
                ptr = 1;
                for kk = 1:length(estm)
                    if( nargin < 5 || valid_idx(kk) )
                        ptr1 = ptr+length(estm{kk})-1;
                        estm{kk} = estm{kk} + dv(ptr:ptr1);
                        ptr = ptr1+1;
                    end
                end
                
                if ((norm(dv)<solver_tol)||(i>=max_iter))
                    done=1;
                    %fprintf('dv too small or max_iter reached\n');
                else
                    %tic;
                    if( nargin > 4 )
                        this.linearize(estm,valid_idx);
                    else
                        this.linearize(estm);
                    end
                    %toc;
                    next_error = norm(this.b);
                    if(next_error<=current_error) && (lambda <= lambda_max)
                        done = (current_error-next_error) < 0.0005; % strange this is here
                        if(done)
                            %fprintf('error change too small\n');
                        end
                        current_error=next_error;
                        % successful step
                        lambda = max(0.1*lambda,eps);
                    else
                        lambda=10*lambda;
                        step_backs=step_backs+1;
                        %fprintf('step backs %d\n',step_backs);
                    end
                end
            end
        end
        
        function mC = get_choimat(this,var_ids)
            % INPUT:
            %   var_ids = cell of col vectors of ids whose cholesky factors
            %       to get. These are ids of the original (not linearized only) variables
            %
            % OUTPUT:
            %   C = cell(size(var_ids)) = cholesky factors for the ids in var_ids
            %
            num_var = length(this.vardim_lin);
            all_var = transpose(1:num_var);
            cvdims = cumsum([0;this.vardim_lin(:)]);
            long_ids = cell(num_var,1);
            for k = 1:num_var
                long_ids{k} = cvdims(k) + transpose(1:this.vardim_lin(k));
            end
            
            % Obtain cholesky factors
            num_get = length(var_ids);
            mC = cell(num_get,1);
            for k = 1:num_get
                % get linearized indices
                var_ids_k = this.all_to_lin_idx(var_ids{k});
                var_ids_k(var_ids_k == 0) = [];
                not_k = setdiff(all_var,var_ids_k);
                
                % get long linearized idx
                long_not_k = cat(1,long_ids{not_k});
                long_yes_k = cat(1,long_ids{var_ids_k});
                sz_no = length(long_not_k);
                sz_yes = length(long_yes_k);
                mC{k} = this.A(:, [long_not_k; long_yes_k]);
                mC{k} = qr(mC{k});
                to_keep = (sz_no+1):(sz_no+sz_yes);
                mC{k} = mC{k}( to_keep, to_keep );
            end
        end
    end
    
    
    methods (Static)
        function dv = solve_iter(A,b,lambda)
            % lambda = Levenberg-Marquadt parameter
            nVar = size(A,2);
            
            toScale = false;
            if toScale
                diagJacTJac = sum(A.^2,1).';
                scaleMat = spdiags(sqrt(lambda*diagJacTJac),0,nVar,nVar);
            else
                scaleMat = sqrt(lambda)*speye(nVar);
            end
            
            AugJac = [A; scaleMat];
            AugRes = [b; sparse(nVar,1)];
            
            
            % solve normal equations
            %dv = AugJac \ AugRes;
            
            % solve via qr factorization 'vector' OR 0
            order = colamd(AugJac); % reorder columns
            AugJac = AugJac(:,order);
            [d,R,P] = qr(AugJac,AugRes,0);        % P = colamd(AugJac);
            dv = R\d;                             % AugJac = AugJac(:,P);
            dv(P) = dv;                           % R = qr([AugJac,AugRes]);
            % d = R(:,end);
            % R = R(:,1:end-1);
            dv(order) = dv; % invert order
        end

        function Sig = get_cov(A)
            % A = square root info matrix
            nVar = size(A,2);
            Hess = (A.'*A);
            Sig = Hess \ speye(nVar);
        end
    end
end








%{
function [jC,mC,jmC] = get_choimat(this,var_dims,var_ids,order)
    % INPUT:
    %   var_dims = dimensions of all variables (before ordering)
    %   var_ids = cell of col vectors of ids to get (before ordering)
    %   order = order the variables in a certain way before
    %   extracting the cholesky factors
    %
    %
    % OUTPUT:
    %   jC = cholesky factor of the joint info matrix of all variables
    %   mC = cell(num_var,1) = marginal cholesky factors for the ids in var_ids
    %   jmC = cell(num_var,1) = accumulates unions of variables and
    %   produces the joint marginal cholesky factors
    %
    %   e.g. var_ids{1} = 1 3 5
    %        var_ids{2} = 1 7 8
    %        jmC = cholesky factor of 1 3 5 7 8
    %

    num_var = length(var_dims);
    all_var = transpose(1:num_var);
    cvdims = cumsum([0;var_dims(:)]);

    long_ids = cell(num_var,1);
    for k = 1:num_var
        long_ids{k} = cvdims(k) + transpose(1:var_dims(k));
    end

    if(nargin > 3)
        long_ids = long_ids(order);
        jC = qr( this.A(:,cat(1,long_ids{:})) );
    else
        order = all_var;
        jC = qr(this.A);
    end

    % Obtain marginals
    num_get = length(var_ids);
    mC = cell(num_get,1);
    jmC = cell(num_get,1);
    for k = 1:num_get
        if( isempty(var_ids{k}) )
            if( k > 1)
                jmC{k} = jmC{k-1};
            end
            continue;
        end
        var_ids_k = order(var_ids{k});
        not_k = setdiff(all_var,var_ids_k);
        long_not_k = cat(1,long_ids{not_k});
        long_yes_k = cat(1,long_ids{var_ids_k});
        sz_no = length(long_not_k);
        sz_yes = length(long_yes_k);
        mC{k} = jC(:, [long_not_k; long_yes_k]);
        mC{k} = qr(mC{k});
        to_keep = (sz_no+1):(sz_no+sz_yes);
        mC{k} = mC{k}( to_keep, to_keep );

        if(k == 1)
            jmC{k} = mC{k};
        else
            cum_var_ids = unique(order(cat(1,var_ids{1:k})),'stable');
            not_k = setdiff(all_var,cum_var_ids);
            long_not_k = cat(1,long_ids{not_k});
            long_yes_k = cat(1,long_ids{cum_var_ids});
            sz_no = length(long_not_k);
            sz_yes = length(long_yes_k);
            jmC{k} = jC(:, [long_not_k; long_yes_k]);
            jmC{k} = qr(jmC{k});
            to_keep = (sz_no+1):(sz_no+sz_yes);
            jmC{k} = jmC{k}( to_keep, to_keep );
        end
    end
end
end
%}