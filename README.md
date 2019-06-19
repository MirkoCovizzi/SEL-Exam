Mirko Covizzi 28/05/2019

Relazione per progetto d'esame Sistemi di Elaborazione

Il progetto consiste nella realizzazione di un benchmark per la board NXP LPC xpresso 43s37 avente cores M4 ed M0APP entrambi a 204MHz.
Il benchmark consiste nel testare due differenti metodi di comunicazione tra i core, uno sincrono utilizzando interrupt ed uno asincrono utilizzando mutex senza l'utilizzo di interrupt.

Descrizione metodo sincrono:

Il core M4 imposta un timer (TIMER0) a frequenza settabile da configurazione. Questo timer viene utilizzato per generare le richieste al core M0APP. Le richieste consistono nella generazione di messaggi contenenti un ID incrementale, un valore di input per la funzione di M0APP ed un timestamp. Una volta generata la richiesta, all'interno dell'handler del timer apposito, il core M4 carica il messaggio sulla coda di messaggi del core M0APP, inviando successivamente un SEV (signal event) al core M0APP in modo da avvisarlo che é stato caricato un messaggio sulla sua coda. Dall'altra parte, il core M0APP, una volta ricevuto il segnale, lo gestisce nell'apposito handler M4_IRQHandler, preleva il messaggio caricato dal core M4, prende il valore di input della richiesta e computa il risultato utilizzando la sua funzione (in questo caso il core M0APP implementa una funzione che controlla se un numero é primo; se lo é restituisce il valore 1, altrimenti 0). Una volta terminata la computazione del risultato, il core M0APP carica il nuovo messaggio con il risultato, conservando stesso ID e timestamp. Invia poi un segnale al core M4, il quale nell'handler M0APP_IRQHandler preleva il messaggio dalla propria coda e determina il tempo trascorso tra la richiesta e la risposta (delta time).
Nel programma é possibile attivare il carico di CPU su entrambi i processori.
Senza carico di CPU, se il tasso di richieste da parte di M4 é sufficientemente basso, il tempo di risposta (delta time) sarà adeguatamente basso. Se invece il tasso di richieste supera una certa soglia, il tempo di risposta aumenterà considerevolmente, poiché, dall'altra parte, il core M0APP riceverà interrupts ad una frequenza ugualmente alta, causando quindi un aumento della latenza generale. Tuttavia, anche con carico di CPU, il tempo di risposta non cambia molto utilizzando questo metodo.

Descrizione metodo asincrono

Come nel metodo precedente, il core M4 imposta il timer per le richieste. Inoltre, sia M4 che M0APP gestiscono due mutex, strutture dati per la mutua esclusione all'accesso ad una risorsa. Un mutex viene utilizzato per garantire la mutua esclusione nell'accesso alla periferica UART per la stampa ed il debug, mentre l'altro mutex viene utilizzato per scrivere e/o leggere sulla stessa area di memoria. Il mutex é costruito utilizzando il paper di Dijkstra contenuto nella directory del progetto, con la sola assunzione che le scritture e letture di 32bit in memoria siano atomiche tra i due core. Come nel metodo precedente, i messaggi di richiesta vengono generati nell'handler del timer. Il core M4 quindi, per ogni richiesta, tenta di acquisire il lock. Una volta acquisito, scrive nell'area di memoria comune il messaggio di richiesta (messaggio strutturato in maniera simile al metodo sincrono), poi rilascia il mutex. Il core M0APP, dall'altra parte, si troverà in un loop infinito nel quale tenterà di acquisire il lock. Una volta acquisito, se il counter del messaggio di richiesta é dispari, allora significa che nell'area di memoria comune troverà un messaggio di richiesta generato da M4. In questo caso, M0APP (ora possessore del lock), leggerà il messaggio, incrementerà il counter, computerà il risultato della richiesta ed aggiornerà di conseguenza il messaggio nell'area di memoria condivisa. Infine, M0APP rilascerà il lock sul mutex. Il core M4, nel suo ciclo principale tenta costantemente di acquisire il lock sul mutex, solo se una flag (received) é uguale a zero. La flag sarà uguale a zero se M4 ha prodotto una richiesta ma non ha ricevuto ancora risposta, altrimenti a 1. Una volta acquisito il lock, il core M4, nel ciclo principale, controllerà se il counter é pari, cioé se il core M0APP ha risposto alla richiesta. In questo caso, il core M4 leggerà il risultato (in caso di debug stamperà la risposta ed il delta time) e successivamente rilascerà il lock sul mutex.
Senza carico di CPU, questo metodo é sempre più veloce del metodo sincrono, anche ad un tasso di richieste alto. Tuttavia, in presenza di carico di CPU, sia il core m4 che il core M0APP saranno nella sezione di polling dell'area comune solo dopo aver eseguito la loro computazione richiesta (il carico di CPU), causando quindi un tasso di polling via via più basso al crescere del carico sulla CPU. In caso di carico di CPU, quindi, i tempi di risposta, oltre una certa soglia di carico di CPU, saranno maggiori (e quindi peggiori) rispetto al caso sincrono.

In conclusione, in generale, poiché solitamente i core hanno sempre un proprio carico di CPU, conviene utilizzare un metodo di comunicazione sincrono, il quale é infatti il metodo suggerito nel datasheet della board stessa.

Descrizione funzioni dei moduli M4 ed M0

M4:

- M0APP_IRQHandler:
Funzione handler che gestisce gli eventi inviati dal core M0 al core M4. Ogni volta che ricevo un evento imposto la variabile volatile intera notify a 1. Se sto eseguendo il test numero uno, cioé comunicazione sincrona ad eventi, allora eseguo il blocco di codice opzionale. Nel blocco di codice opzionale il core M4 tenta di estrarre il messaggio dalla propria coda. Calcola inoltre il delta time facendo la differenza tra il valore del contatore di clock attuale ed il valore contenuto nel messaggio.
- TIMER0_IRQHandler:
Funzione handler per gestire il timer 0. Il timer 0, in questo caso, mi serve per generare azioni periodiche. In modo specifico, per inviare periodicamente messaggi al core M0. In questo handler ho due blocchi opzionali: se il test é il test numero 1, allora confeziono un messaggio da inviare utilizzando le code e quindi il metodo sincrono; se il test é il numero 2, confeziono un messaggio per il core M0, entro nella sezione critica con accesso atomico tramite mutex, scrivo il messaggio nell'area di memoria comune, rilascio il mutex uscendo dalla sezione critica.
- TIMER1_IRQHandler:
Semplice contatore temporale
- wait_event:
Implementazione di una wait. In sostanza, il core si blocca finché non riceve una notify dall'altro core.
- notify_event:
Implementazione di una notify. Il core che invoca questa funzione manda un segnale all'altro core.
- init_mutex:
Inizializzazione del mutex descritto nel paper di Dijkstra
- lock_mutex:
Implementazione della acquisizione del mutex, algoritmo descritto nel paper di Dijkstra.
- unlock_mutex:
Rilascio del mutex
- print:
Implementazione di una stampa mutuamente esclusiva. Non posso avere due stampe in contemporanea tra i due core.
- debug:
Come la print.
- print_menu:
Stampa delle opzioni del benchmark.
- setup_board:
Inizializzazione della board.
- boot_M0App:
Funzione che avvia il core M0 secondo le specifiche indicate nel datasheet.
- shutdown_M0App:
Spengo il core M0.
- setup_uart:
Inizializzazione di base di una uart per le stampe.
- setup_clock_counter:
Inizializzazione del contatore di clock tramite registro DWT (presente solo su core M4).
- clock_counter_value:
Funzione che restituisce il valore del contatore del ciclo di clock.
- setup_system_timer:
Abilita timer che conta i secondi.
- stop_system_timer:
Disabilita il teimer che conta i secondi.
- setup_request_timer:
Inizializzazione timer per le richieste al core M0
- start_request_timer:
Abilita timer per le richieste.
- stop_request_timer:
Disabilita timer per le richieste.
- calc_pi:
Funzione per caricare la CPU di calcolo.
- main:
Funzione principale: qui, tramite switch, si passa alle subroutine dei vari test. Ogni test viene inizializzato dal core M4, e tramite comunicazione sincrona il core M0 viene impostato dal core M4 per poter fare il test selezionato.
- idle:
Funzione che calcola l'utilizzo di CPU.

M0:

- M4_IRQHandler:
Funzione handler che gestisce gli eventi inviati dal core M4 al core M0. Ogni volta che ricevo un evento imposto la variabile volatile intera notify a 1. Se sto eseguendo il test numero uno, cioé comunicazione sincrona ad eventi, allora eseguo il blocco di codice opzionale. Nel blocco di codice opzionale il core M0 tenta di estrarre il messaggio dalla propria coda. Calcola inoltre il risultato della richiesta ricevuta generando poi un nuovo messaggio (con il risultato) ed inviandolo al core M4.
- TIMER3_IRQHandler:
Semplice contatore temporale
- wait_event:
Implementazione di una wait. In sostanza, il core si blocca finché non riceve una notify dall'altro core.
- notify_event:
Implementazione di una notify. Il core che invoca questa funzione manda un segnale all'altro core.
- init_mutex:
Inizializzazione del mutex descritto nel paper di Dijkstra
- lock_mutex:
Implementazione della acquisizione del mutex, algoritmo descritto nel paper di Dijkstra.
- unlock_mutex:
Rilascio del mutex
- print:
Implementazione di una stampa mutuamente esclusiva. Non posso avere due stampe in contemporanea tra i due core.
- debug:
Come la print.
- print_menu:
Stampa delle opzioni del benchmark.
- setup_board:
Inizializzazione della board.
- boot_M0App:
Funzione che avvia il core M0 secondo le specifiche indicate nel datasheet.
- shutdown_M0App:
Spengo il core M0.
- setup_uart:
Inizializzazione di base di una uart per le stampe.
- setup_clock_counter:
Inizializzazione del contatore di clock tramite registro DWT (presente solo su core M4).
- clock_counter_value:
Funzione che restituisce il valore del contatore del ciclo di clock.
- setup_system_timer:
Abilita timer che conta i secondi.
- stop_system_timer:
Disabilita il teimer che conta i secondi.
- check_prime:
Funzione per caricare la CPU di calcolo.
- main:
Funzione principale: qui il core M0 viene configurato dal core M4 ed entra in uno dei test possibili. Al termine del test il core M4 si occuperà di spegnere il core M0. Il core M0, perciò viene avviato e poi spento per ogni test. Da sottolineare, nel test 2, prima di richiedere il mutex entrando nella sezione critica, bisogna sempre disattivare gli interrupt (tramite __disable_irq) e riattivarli dopo il rilascio del mutex, altrimenti, se ho acquisito il mutex e vengo interrotto da un interrupt, potrebbero verificarsi problemi di concorrenza tra i processori. Questa é una pratica a volte usata nei sistemi operativi.
- idle:
Funzione che calcola l'utilizzo di CPU.