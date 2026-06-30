// Command definitions for each board.
// Each command: { cmd: "<serial string>", label: "Button text", danger?: true, accent?: true }
window.BOARDS = [
  {
    id: 'mixer',
    name: 'Mixer',
    baud: 115200,
    groups: [
      {
        title: 'Movement',
        commands: [
          { cmd: 'mu',      label: 'Mixer Up' },
          { cmd: 'md',      label: 'Mixer Down' },
          { cmd: 'home',    label: 'Home Height' },
          { cmd: 'home2',   label: 'Home 2' },
          { cmd: 'homeall', label: 'Home All' },
          { cmd: 'mix',     label: 'Go to Mix' },
          { cmd: 'raise',   label: 'Go to Raise' },
          { cmd: 'dispense',label: 'Dispense', accent: true },
          { cmd: 'finish',  label: 'Finish', accent: true }
        ]
      },
      {
        title: 'Power & Control',
        commands: [
          { cmd: 'start',    label: 'Relay ON' },
          { cmd: 'end',      label: 'Relay OFF' },
          { cmd: 'poweron',  label: 'Power ON' },
          { cmd: 'poweroff', label: 'Power OFF' },
          { cmd: 'savecal',  label: 'Save Calibration' },
          { cmd: 'stop',     label: 'STOP', danger: true }
        ]
      },
      {
        title: 'Info',
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
        title: 'Movement',
        commands: [
          { cmd: 'home',     label: 'Home' },
          { cmd: 'mix',      label: 'Go to Mix' },
          { cmd: 'dispense', label: 'Dispense', accent: true }
        ]
      },
      {
        title: 'Jog',
        commands: [
          { cmd: 'f',  label: 'Main Forward' },
          { cmd: 'b',  label: 'Main Back' },
          { cmd: 'bf', label: 'Bowl Forward' },
          { cmd: 'bb', label: 'Bowl Back' }
        ]
      },
      {
        title: 'Magnet',
        commands: [
          { cmd: 'on',     label: 'Magnet ON' },
          { cmd: 'off',    label: 'Magnet OFF' },
          { cmd: 'toggle', label: 'Toggle Magnet' }
        ]
      },
      {
        title: 'Power & Control',
        commands: [
          { cmd: 'poweron',  label: 'Power ON' },
          { cmd: 'poweroff', label: 'Power OFF' },
          { cmd: 'stop',     label: 'STOP', danger: true }
        ]
      },
      {
        title: 'Info',
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
    name: 'Linear',
    baud: 115200,
    groups: [
      {
        title: 'Movement',
        commands: [
          { cmd: 'home',     label: 'Home' },
          { cmd: 'left',     label: 'Left' },
          { cmd: 'right',    label: 'Right' },
          { cmd: 'centre',   label: 'Centre' },
          { cmd: 'mix',      label: 'Go to Mix' },
          { cmd: 'dispense', label: 'Dispense', accent: true },
          { cmd: 'stop',     label: 'STOP', danger: true },
          { cmd: 'pos',      label: 'Position' }
        ]
      },
      {
        title: 'Dispenser',
        commands: [
          { cmd: 'start',        label: 'Start Dispensing', accent: true },
          { cmd: 'stopdispense', label: 'Stop Dispensing', danger: true },
          { cmd: 't',            label: 'Tare' },
          { cmd: 'weight',       label: 'Read Weight' },
          { cmd: 'weightstream', label: 'Weight Stream ON' },
          { cmd: 'weightoff',    label: 'Weight Stream OFF' }
        ]
      },
      {
        title: 'Gate Servo',
        commands: [
          { cmd: 'open',  label: 'Gate Open' },
          { cmd: 'close', label: 'Gate Close' },
          { cmd: 'auto',  label: 'Gate Auto' }
        ]
      },
      {
        title: 'Manifold Servos',
        commands: [
          { cmd: 'o',  label: 'Mixer Extraction Servo Open' },
          { cmd: 'c',  label: 'Mixer Extraction Servo Close' },
          { cmd: 'o2', label: 'Servo B Open' },
          { cmd: 'c2', label: 'Servo B Close' }
        ]
      },
      {
        title: 'Vacuum',
        commands: [
          { cmd: 'vacuumon',  label: 'Vacuum ON' },
          { cmd: 'vacuumoff', label: 'Vacuum OFF' }
        ]
      },
      {
        title: 'Hall & Power',
        commands: [
          { cmd: 'hall',       label: 'Hall State' },
          { cmd: 'hallstream', label: 'Hall Stream ON' },
          { cmd: 'halloff',    label: 'Hall Stream OFF' },
          { cmd: 'poweron',    label: 'Power ON' },
          { cmd: 'poweroff',   label: 'Power OFF' },
          { cmd: 'toggle',     label: 'Toggle Power' },
          { cmd: 'status',     label: 'Status' }
        ]
      }
    ]
  }
];
