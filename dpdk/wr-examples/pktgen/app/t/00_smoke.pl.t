#!/usr/bin/env perl

use strict;
use warnings;

use Test::More;

BEGIN {
    if ($ENV{PKTGEN_CMDLINE}) {
        plan tests => 3;
    }
    else {
        plan skip_all => 'Environment variable $PKTGEN_CMDLINE must be set to a valid Pktgen command line';
    }
}


use IPC::Open3;
use POSIX ":sys_wait_h";

my $success = 1;
my $pid;

# Redirect Pktgen STDIN, STDOUT, STDERR to strings
use Symbol 'gensym';
my ($in, $out, $err) = (undef, undef, gensym());

eval {
    $pid = open3($in, $out, $err, 'sudo ../build/app/pktgen ' . $ENV{PKTGEN_CMDLINE});
};
if ($@) {
    $success = 0;
}

ok($success, 'Pktgen must start');

diag "Waiting 30 seconds for Pktgen to start...";
sleep 30;

$in .= "quit\n";
diag "Waiting 5 more seconds for Pktgen to quit";
sleep 5;

my $kid = waitpid $pid, WNOHANG;

is($kid, $pid, 'Pktgen must quit when the \'quit\' command is given');
is($? >> 8, 0, '... and return success');

done_testing();
