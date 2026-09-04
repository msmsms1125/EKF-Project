function [ cross_point ] = mch_line_cross_point( line1_base, line1_dir, line2_base, line2_dir )

    A = [line1_dir, -line2_dir];
    B = line2_base - line1_base;
    
    C = A\B;
    
    cross_point = line1_base + C(1) * line1_dir;

end

