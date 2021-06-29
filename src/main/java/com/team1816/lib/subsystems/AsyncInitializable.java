package com.team1816.lib.subsystems;

import java.util.concurrent.CompletableFuture;

public interface AsyncInitializable {
    CompletableFuture<Void> initAsync();
}
