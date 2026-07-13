// Command definitions for each board.
// Each command: { cmd: "<serial string>", label: "Button text", danger?: true, accent?: true }
window.BOARDS = [
  {
    id: 'mixer',
    name: 'Mixer',
    baud: 115200,
    groups: [
      {
        title: 'Power',
        commands: [
          { cmd: 'poweron',  label: 'Power On' },
          { cmd: 'poweroff', label: 'Power Off' }
        ]
      },
      {
        title: 'Home',
        commands: [
          { cmd: 'homeall', label: 'Home All' },
          { cmd: 'home',    label: 'Home Mixer' }
        ]
      },
      {
        title: 'Movement',
        commands: [
          { cmd: 'mix',               label: 'Move to Mix Position' },
          { cmd: 'raise',             label: 'Move to Raise Position' },
          { cmd: 'refillcement',      label: 'Refill Cement' },
          { cmd: 'mixcycle',          label: 'Mix' },
          { cmd: 'dispense',          label: 'Dispense' },
          { cmd: 'addmortar',         label: 'Add Mortar' },
          { cmd: 'returnafterdispense', label: 'Return After Dispense' },
          { cmd: 'finish',            label: 'Finish' }
        ]
      },
      {
        title: 'Analysis',
        commands: [
          { cmd: 'start', label: 'Start Bingham Analysis' },
          { cmd: 'end',   label: 'End Bingham Analysis' }
        ]
      },
      {
        title: 'Manual Control',
        commands: [
          { cmd: 'mu', label: 'Raise Mixer' },
          { cmd: 'md', label: 'Lower Mixer' }
        ]
      },
      {
        title: 'Information',
        commands: [
          { cmd: 'pos',  label: 'Position' },
          { cmd: 'hall', label: 'Hall State' }
        ]
      }
    ]
  },

  {
    id: 'bowl',
    name: 'Bowl',
    baud: 9600,
    groups: [
      {
        title: 'Power',
        commands: [
          { cmd: 'poweron',  label: 'Power On' },
          { cmd: 'poweroff', label: 'Power Off' }
        ]
      },
      {
        title: 'Commands',
        commands: [
          { cmd: 'home',     label: 'Home' },
          { cmd: 'mix',      label: 'Move to Mix Position' },
          { cmd: 'dispense', label: 'Dispense' }
        ]
      },
      {
        title: 'Manual Control',
        commands: [
          { cmd: 'b',      label: 'Lift Front Brim' },
          { cmd: 'f',      label: 'Lift Back Brim' },
          { cmd: 'bf',     label: 'Bowl Backwards' },
          { cmd: 'bb',     label: 'Bowl Forwards' },
          { cmd: 'on',     label: 'Bowl Rotate On' },
          { cmd: 'off',    label: 'Bowl Rotate Off' },
          { cmd: 'toggle', label: 'Toggle Magnet' }
        ]
      },
      {
        title: 'Information',
        commands: [
          { cmd: 'pos',     label: 'Position' },
          { cmd: 'status',  label: 'Status' },
          { cmd: 'hall',    label: 'Hall State' },
          { cmd: 'halloff', label: 'Hall Stream Off' }
        ]
      }
    ]
  },

  {
    id: 'linear',
    name: 'Conveyor',
    baud: 115200,
    groups: [
      {
        title: 'Power',
        commands: [
          { cmd: 'poweron',  label: 'Power On' },
          { cmd: 'poweroff', label: 'Power Off' }
        ]
      },
      {
        title: 'Commands',
        commands: [
          { cmd: 'home',         label: 'Home' },
          { cmd: 'mix',          label: 'Move to Mix Position' },
          { cmd: 'start',        label: 'Start Dispensing' },
          { cmd: 'stopdispense', label: 'Stop Dispensing' }
        ]
      },
      {
        title: 'Manual Control',
        commands: [
          { cmd: 'left',  label: 'Left' },
          { cmd: 'right', label: 'Right' },
          { cmd: 'pos',   label: 'Position' }
        ]
      },
      {
        title: 'Dispenser',
        commands: [
          { cmd: 't',            label: 'Tare' },
          { cmd: 'weight',       label: 'Read Weight' },
          { cmd: 'weightstream', label: 'Weight Stream On' },
          { cmd: 'weightoff',    label: 'Weight Stream Off' }
        ]
      },
      {
        title: 'Gate Servo',
        commands: [
          { cmd: 'open',  label: 'Gate Open' },
          { cmd: 'close', label: 'Gate Close' }
        ]
      },
      {
        title: 'Manifold Servos',
        commands: [
          { cmd: 'o',  label: 'Mixer Extraction Servo Open' },
          { cmd: 'c',  label: 'Mixer Extraction Servo Close' },
          { cmd: 'o2', label: 'Dispenser Servo Open' },
          { cmd: 'c2', label: 'Dispenser Servo Close' }
        ]
      },
      {
        title: 'Vacuum',
        commands: [
          { cmd: 'vacuumon',  label: 'Vacuum On' },
          { cmd: 'vacuumoff', label: 'Vacuum Off' }
        ]
      },
      {
        title: 'Hall & Power',
        commands: [
          { cmd: 'hall',       label: 'Hall State' },
          { cmd: 'hallstream', label: 'Hall Stream On' },
          { cmd: 'halloff',    label: 'Hall Stream Off' },
          { cmd: 'status',     label: 'Status' }
        ]
      }
    ]
  },

  {
    id: 'linear2',
    name: 'Overflow',
    baud: 115200,
    groups: [
      {
        title: 'Commands',
        commands: [
          { cmd: 'raise', label: 'Raise' },
          { cmd: 'lower', label: 'Lower' },
          { cmd: 'stop',  label: 'Stop' }
        ]
      }
    ]
  }
];
