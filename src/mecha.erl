
%%% THE EVERYTHING PATH
-module(mecha).
-compile(export_all).

-define(ECAPNODE, 'c1@localhost').


mecha(InterruptSource) ->
    {any, ?ECAPNODE} ! { call, self(), { init, InterruptSource } },
    receive
        { ecap_node, ok } ->
            hes_loop();
        Err ->
            io:format("init call to ecap node failed: ~w~n", [Err]),
            error(Err)
    end.

hes_loop(CapRegFile, Phase) ->
    receive
        { data, Data } ->
            Regs = do_ecap(Data, CapRegFile),
            {SyncTooth, PriorTooth, Tw, T1} = do_mle()
            loop();
        { getphase , Whom } ->
    end.

do_ecap(Data, CapRegFile) ->
    %% Data is ecap_regs->ECFLG
    EcapFlg = list_to_integer(Data, 16),
    {ok, CapStr} = file:read_file(CapRegFile),
    CapTok = string:tokens(binary_to_list(CapStr), ","),
    Stamps = [ list_to_integer(T, 16) || T <- CapTok ],
    Regs = [ { Tw, T1 } || {Tw, T1 } <- ecap:get_tstamp(Stamps, EcapFlg) ],
    Regs.

mle_wrap(ToothDist, PriorStamp, T1, MaxAccel, ErrRate, ToothProb, [NewReg | Regs ]) ->
    
    

do_mle(ToothDist, PriorStamp, T1, MaxAccel, ErrRate, ToothProb) ->
    MovedProb = move(ErrRate, ToothProb),
    LocatedProb = locate(ToothDist, PriorStamp, T1, MaxAccel, ErrRate, ToothProb)
    Located = locate(Moved, T1),
    Normalized = normalize(Located),
    Confidence = lists:max(Normalized#mlestate.toothprob),
    Normalized#mlestate{ syncconf   = Confidence,
                         priorstamp = T1,
                         synctooth  = tools:index_of(Confidence, Normalized#mlestate.toothprob) }.
    ErrAcc = calc_accel(
                lists:nth( Prior#mlestate.synctooth, Prior#mlestate.toothdist         ), Prior#mlestate.priorstamp,
                lists:nth( Posterior#mlestate.synctooth, Posterior#mlestate.toothdist ), T1,
                lists:sum(Prior#mlestate.toothdist) 
             ),
    case { Posterior, ErrAcc } of
        { S = #mlestate{ maxaccel=MaxAccel }, E } when E >= MaxAccel -> 
            gen_event:notify( S#mlestate.whom, { nosync } ),
            { ok, S#mlestate{ hassync = false } };
        { S = #mlestate{ syncconf=SyncConf, minconf=MinConf }, _ } when SyncConf >= MinConf ->
            gen_event:notify( S#mlestate.whom, { sync, { S#mlestate.synctooth, Prior#mlestate.synctooth, Tw, T1 } } ),
            { ok, S#mlestate{ hassync = true } };
        { S = #mlestate{ hassync=true }, _ } ->
            gen_event:notify( S#mlestate.whom, { sync, { S#mlestate.synctooth, Prior#mlestate.synctooth, Tw, T1 } } ),
            { ok, S };
        { S, _ } ->
            gen_event:notify( S#mlestate.whom, { nosync } ),
            { ok, S }
