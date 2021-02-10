// fsm-config: {"font" : "Arial"}
// fsm-config: {"nodeShape" : "diamond"}
// fsm-config: {"initialShape" : "component", "finalShape" : "tab"}

let myFsm = {
    initial: 'START',
    final: 'END',
    events: [
        {name: '', from: 'START', to: 'IDLE', style: 'dotted'},
        {name: 'YES', from: 'IDLE', to: 'CHECK', style: 'dotted'},
        {name: 'YES', from: 'CHECK', to: 'C_Temperature', style: 'dotted'},
        {name: 'XXX', from: 'C_Temperature', to: 'GROUP', style: 'dotted'},
        {name: 'XXX', from: 'GROUP', to: 'BALANCE', style: 'dotted'},
        {name: 'XXX', from: 'BALANCE', to: 'END', style: 'dotted'},
    ],
    states: [
        {name: 'IDLE', color: 'green', comments:"unbalanced Cells?"},
        {name: 'CHECK', color: 'green', comments:"balancing allowed?"},
        {name: 'CHECK', color: 'green', comments:"balancing allowed?"},
        {name: 'C_Temperature', color: 'green', comments:"Temperature ok?"},
        {name: 'CHECK', color: 'green', comments:"balancing allowed?"},
        
        {name: 'ORDER', color: 'green', comments:"order cells"},
        {name: 'GROUP', color: 'green', comments:"group cells"},
        {name: 'BALANCE', color: 'green', comments:"balance cells"}
        
        
    ],
    callbacks: {
        //onhome: ()=>{},
        //onend: function watchTV() {}
    }
}