////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Generate the NRD file
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Open file and write the header

fid=mopen('NrdGen.h','w');
mfprintf(fid, '/* -------------------------------------------------------------------------------- */\n');
mfprintf(fid, '/* Generated NRD file */\n')
mfprintf(fid, '/* -------------------------------------------------------------------------------- */\n');
mfprintf(fid, '\n');

////////////////////////////////////////////////////////////////////////////////
// Write data

mfprintf(fid, '/* -------------------------------------------------------------------------------- */\n');
mfprintf(fid, '/* Modulator */\n');
mfprintf(fid, '\n');
torsor = ['FX';'FY';'FZ';'TX';'TY';'TZ'];
for iMotor = 1:Nmotor
    line = "";
    for iDof = 1:6
        name = msprintf("K_MOD_IM_%s%d", torsor(iDof,:), iMotor-1);
        line = line + name + ", ";
        mfprintf(fid, '#define %s\t(%d)\n', name, nrd.modulator.inflMat(iDof, iMotor));
    end
end
mfprintf(fid, '#define K_MOD_INFLMAT {\\\n');
for iMotor = 1:Nmotor
    mfprintf(fid, '\t\t{K_MOD_INFLMAT_FX%d, K_MOD_INFLMAT_FY%d, K_MOD_INFLMAT_FZ%d, K_MOD_INFLMAT_TX%d, K_MOD_INFLMAT_TY%d, K_MOD_INFLMAT_TZ%d}, \\\n', (iMotor-1)*ones(1,6), nrd.modulator.inflMat(:, iMotor)');
end
mfprintf(fid, '\t}\n');
mfprintf(fid, '\n');


// CAPA
for iAxis = 1:3
    mfprintf(fid, '#define K_MOD_FMAXPOS_%d\t(%d)\n',iAxis-1,nrd.modulator.frcMaxPos_B(iAxis));
end
mfprintf(fid, '#define K_MOD_FMAXPOS \\\n');
mfprintf(fid, '\t{K_MOD_FMAXPOS_0, K_MOD_FMAXPOS_1, K_MOD_FMAXPOS_2}\n');
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_MOD_FMAXNEG_%d\t(%d)\n',iAxis-1,nrd.modulator.frcMaxNeg_B(iAxis));
end
mfprintf(fid, '#define K_MOD_FMAXNEG \\\n');
mfprintf(fid, '\t{K_MOD_FMAXNEG_0, K_MOD_FMAXNEG_1, K_MOD_FMAXNEG_2}\n');
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_MOD_TMAXPOS_%d\t(%d)\n',iAxis-1,nrd.modulator.trqMaxPos_B(iAxis));
end
mfprintf(fid, '#define K_MOD_TMAXPOS \\\n');
mfprintf(fid, '\t{K_MOD_TMAXPOS_0, K_MOD_TMAXPOS_1, K_MOD_TMAXPOS_2}\n');
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_MOD_TMAXNEG_%d\t(%d)\n',iAxis-1,nrd.modulator.trqMaxNeg_B(iAxis));
end
mfprintf(fid, '#define K_MOD_TMAXNEG \\\n');
mfprintf(fid, '\t{K_MOD_TMAXNEG_0, K_MOD_TMAXNEG_1, K_MOD_TMAXNEG_2}\n');
mfprintf(fid, '\n');

// LUT
SIGN = ['M' 'P']
for iSign = 1:2
    for iAxis = 1:3
        for iMotor = 1:Nmotor
            mfprintf(fid, '#define K_MOD_LUT_%s%s%d (%d)\n', torsor(iAxis+3,:), SIGN(iSign), iMotor-1, nrd.modulator.lutTrq(iMotor, iAxis, iSign));
        end
    end
end
mfprintf(fid, '#define K_MOD_LUTTRQ {\\\n');
for iSign = 1:2
    mfprintf(fid, '\t\t{ \\\n');
    for iAxis = 1:3
        mfprintf(fid, '\t\t\t{');
        for iMotor = 1:Nmotor
            mfprintf(fid, 'K_MOD_LUT_%s%s%d, ', torsor(iAxis+3,:), SIGN(iSign), iMotor-1);
        end
        mfprintf(fid, '}, \\\n');
    end
    mfprintf(fid, '\t\t}, \\\n');
end
mfprintf(fid, '\t}\n');
mfprintf(fid, '\n');

for iSign = 1:2
    for iAxis = 1:3
        for iMotor = 1:Nmotor
            mfprintf(fid, '#define K_MOD_LUT_%s%s%d (%d)\n', torsor(iAxis,:), SIGN(iSign), iMotor-1, nrd.modulator.lutFrc(iMotor, iAxis, iSign));
        end
    end
end
mfprintf(fid, '#define K_MOD_LUTFRC {\\\n');
for iSign = 1:2
    mfprintf(fid, '\t\t{ \\\n');
    for iAxis = 1:3
        mfprintf(fid, '\t\t\t{');
        for iMotor = 1:Nmotor
            mfprintf(fid, 'K_MOD_LUT_%s%s%d, ', torsor(iAxis,:), SIGN(iSign), iMotor-1);
        end
        mfprintf(fid, '}, \\\n');
    end
    mfprintf(fid, '\t\t}, \\\n');
end
mfprintf(fid, '\t}\n');
mfprintf(fid, '\n');

////////////////////////////////////////////////////////////////////////////////
// Attitude

for iAxis = 1:3
    mfprintf(fid, '#define K_ATTCTRL_KP_%d %.16e\n', iAxis-1, nrd.attitude.Kp(iAxis));
end
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_ATTCTRL_KD_%d %.16e\n', iAxis-1, nrd.attitude.Kd(iAxis));
end
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_ATTCTRL_KI_%d %.16e\n', iAxis-1, nrd.attitude.Ki(iAxis));
end
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_ATTCTRL_MAXI_%d %.16e\n', iAxis-1, nrd.attitude.maxI(iAxis));
end
mfprintf(fid, '\n');

mfprintf(fid, '#define K_ATTCTRL_MAXERRCOS %.16e\n',nrd.attitude.maxCosAngOverTwoErr);
mfprintf(fid, '\n');

mfprintf(fid, '#define K_ATTCTRL_MAXERRSIN %.16e\n',nrd.attitude.maxSinAngOverTwoErr);
mfprintf(fid, '\n');


for idx = 0:(length(nrd.attitude.filterX.num)-1)
    mfprintf(fid, '#define K_ATTCTRL_FILT_X_NUM_%d %.16e\n', idx, nrd.attitude.filterX.num(length(nrd.attitude.filterX.num)-idx));
    mfprintf(fid, '#define K_ATTCTRL_FILT_X_DEN_%d %.16e\n', idx, nrd.attitude.filterX.den(length(nrd.attitude.filterX.num)-idx));
end
mfprintf(fid, '#define K_ATTCTRL_FILT_X_NUM {K_ATTCTRL_FILT_X_NUM_0, K_ATTCTRL_FILT_X_NUM_1, K_ATTCTRL_FILT_X_NUM_2}\n');
mfprintf(fid, '#define K_ATTCTRL_FILT_X_DEN {K_ATTCTRL_FILT_X_DEN_0, K_ATTCTRL_FILT_X_DEN_1, K_ATTCTRL_FILT_X_DEN_2}\n');
mfprintf(fid, '\n');

for idx = 0:(length(nrd.attitude.filterY.num)-1)
    mfprintf(fid, '#define K_ATTCTRL_FILT_Y_NUM_%d %.16e\n', idx, nrd.attitude.filterY.num(length(nrd.attitude.filterY.num)-idx));
    mfprintf(fid, '#define K_ATTCTRL_FILT_Y_DEN_%d %.16e\n', idx, nrd.attitude.filterY.den(length(nrd.attitude.filterY.num)-idx));
end
mfprintf(fid, '#define K_ATTCTRL_FILT_Y_NUM {K_ATTCTRL_FILT_Y_NUM_0, K_ATTCTRL_FILT_Y_NUM_1, K_ATTCTRL_FILT_Y_NUM_2}\n');
mfprintf(fid, '#define K_ATTCTRL_FILT_Y_DEN {K_ATTCTRL_FILT_Y_DEN_0, K_ATTCTRL_FILT_Y_DEN_1, K_ATTCTRL_FILT_Y_DEN_2}\n');
mfprintf(fid, '\n');

for idx = 0:(length(nrd.attitude.filterZ.num)-1)
    mfprintf(fid, '#define K_ATTCTRL_FILT_Z_NUM_%d %.16e\n', idx, nrd.attitude.filterZ.num(length(nrd.attitude.filterZ.num)-idx));
    mfprintf(fid, '#define K_ATTCTRL_FILT_Z_DEN_%d %.16e\n', idx, nrd.attitude.filterZ.den(length(nrd.attitude.filterZ.num)-idx));
end
mfprintf(fid, '#define K_ATTCTRL_FILT_Z_NUM {K_ATTCTRL_FILT_Z_NUM_0, K_ATTCTRL_FILT_Z_NUM_1, K_ATTCTRL_FILT_Z_NUM_2}\n');
mfprintf(fid, '#define K_ATTCTRL_FILT_Z_DEN {K_ATTCTRL_FILT_Z_DEN_0, K_ATTCTRL_FILT_Z_DEN_1, K_ATTCTRL_FILT_Z_DEN_2}\n');
mfprintf(fid, '\n');

////////////////////////////////////////////////////////////////////////////////
// Navigation

for iAxis = 1:3
    mfprintf(fid, '#define K_NAVCTRL_KP_%d %f\n', iAxis-1, nrd.navigation.Kp(iAxis));
end
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_NAVCTRL_KD_%d %f\n', iAxis-1, nrd.navigation.Kd(iAxis));
end
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_NAVCTRL_KI_%d %f\n', iAxis-1, nrd.navigation.Ki(iAxis));
end
mfprintf(fid, '\n');

for iAxis = 1:3
    mfprintf(fid, '#define K_NAVCTRL_MAXI_%d %f\n', iAxis-1, nrd.navigation.maxI(iAxis));
end
mfprintf(fid, '\n');

////////////////////////////////////////////////////////////////////////////////
// Write the footer and close the file

mclose(fid);
