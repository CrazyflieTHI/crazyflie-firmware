SECTIONS
{
	.param : {
        PROVIDE(_param_start = .);
        KEEP(*(.param))
        KEEP(*(.param.*))
        PROVIDE(_param_stop = .);
    }
    .log : {
        PROVIDE(_log_start = .);
        KEEP(*(.log))
        KEEP(*(.log.*))
        PROVIDE(_log_stop = .);
    }
    .nop : {
        PROVIDE(_sidata = .);
        PROVIDE(_sdata = .);
        PROVIDE(_edata = .);
        PROVIDE(_stext = .);
        PROVIDE(_etext = .);
    }
	.eventtrigger : {
        PROVIDE(_eventtrigger_start = .);
        KEEP(*(.eventtrigger))
        KEEP(*(.eventtrigger.*))
        PROVIDE(_eventtrigger_stop = .);
    }
}
INSERT AFTER .text;