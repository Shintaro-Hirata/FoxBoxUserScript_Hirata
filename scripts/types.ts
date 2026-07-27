// ============================================================================
// LOCAL TOOLING STUB — DO NOT PASTE INTO THE FOXBOX / FOXGLOVE EDITOR.
//
// Foxglove(FoxBox) provides its own "./types" module to User Scripts at runtime.
// This stub exists ONLY so that the scripts in this folder can be type-checked
// (`npm run typecheck`) and executed by the behavior tests (`npm test`) outside
// of Foxglove. It intentionally types `message` as `unknown`; each script casts
// the message via `as unknown as { ... }`.
// ============================================================================

export type Time = { sec: number; nsec: number };

// The real Foxglove type maps each subscribed topic name to its generated
// message type. For local tooling we only need the event shape, so any topic
// string resolves to an `unknown` message.
interface Messages {
  [topic: string]: unknown;
}

export type Input<T extends keyof Messages = string> = {
  topic: T;
  receiveTime: Time;
  message: Messages[T];
};
