%%% Analog interface module

%%% Right now this is literally just a stub until I figure how how/where
%%%  I actually want to do ADC, and whether I'll drive ADC reads with
%%%  interrupts or what. Since ADC reads are usually asynchronous in the
%%%  SOC, and relatively slow (compared to, say, the crank trigger), I will
%%%  probably end up doing the dumbest/simplest thing possible and just provide
%%%  a means for on-demand reads from within whatever main loop I have.

-module(adc).
-behaviour(gen_server).

-export([start_adc/0, add_channel/1, close_channel/1, read_channel/1]).
-export([init/1, handle_cast/2, handle_call/3]).

%% ----
%%  API
%% ----

start_adc() ->
    gen_server:start_link({local, adc}, adc, [], []).

add_channel(Channel) ->
    gen_server:cast(adc, {add_ch, Channel}).

close_channel(Channel) ->
    gen_server:cast(adc, {rem_ch, Channel}).

read_channel(Channel) ->
    gen_server:call(adc, {read, Channel}).

%% ----
%%  gen_server callbacks
%% ----

init(_Args) ->
    {ok, []}.

handle_cast({add_ch, Channel}, State) ->
    % initialize the ADC channel, as necessary
    % for now, pretend.
    case lists:member(Channel, State) of
        true -> {noreply, State};
        false -> {noreply, [Channel|State]}
    end;
handle_cast({rem_ch, Channel}, State) ->
    % stop the ADC channel, as necessary
    % for now, pretend
    case lists:member(Channel, State) of
        true -> {noreply, State};
        false -> {noreply, lists:delete(Channel, State)}
    end.

handle_call({read, Channel}, _From, State) ->
    % pretend to read the ADC
    case lists:member(Channel, State) of
        true -> {reply, random:uniform() * Channel, State};
        false -> {reply, channel_not_ready, State}
    end.