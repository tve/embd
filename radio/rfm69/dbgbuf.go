// Copyright by Thorsten von Eicken 2016, see LICENSE file

// This file implements a debug buffer into which events can be pushed and the later printed from
// for debugging purposes.

package rfm69

import (
	"fmt"
	"sync"
	"time"
)

type dbgEvent struct {
	at  time.Time
	txt string
}

var dbgBuf = []dbgEvent{}
var dbgMutex = sync.Mutex{}

func dbgPush(txt string) { dbgPushAt(time.Now(), txt) }
func dbgPushAt(at time.Time, txt string) {
	dbgMutex.Lock()
	defer dbgMutex.Unlock()
	dbgBuf = append(dbgBuf, dbgEvent{at, txt})
}

func dbgPrint() {
	dbgMutex.Lock()
	defer dbgMutex.Unlock()

	if len(dbgBuf) == 0 {
		fmt.Printf("No events were recorded\n")
		return
	}

	t0 := dbgBuf[0].at
	for _, ev := range dbgBuf {
		fmt.Printf("%.6fs: %s\n", ev.at.Sub(t0).Seconds(), ev.txt)
	}

	dbgBuf = []dbgEvent{}
}
